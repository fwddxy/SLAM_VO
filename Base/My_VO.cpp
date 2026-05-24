#include "My_VO.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <future>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

using namespace cv;
using namespace std;

namespace {

constexpr int kTargetFeatureCount = 2000;
constexpr int kMinTrackedFeatures = 1000;
constexpr double kMinTranslationScale = 0.1;
constexpr double kDefaultImageScale = 1.0;
constexpr int kTraceCanvasSize = 1000;
constexpr int kProgressReportInterval = 50;
constexpr double kTracePadding = 40.0;
constexpr float kMaxForwardBackwardError = 1.0f;
constexpr int kMinInitialPoseInliers = 8;
constexpr int kMinPoseInliers = 20;

namespace fs = std::filesystem;

fs::path findProjectRoot() {
    fs::path current = fs::current_path();
    while (!current.empty()) {
        if (fs::exists(current / "Kitti") && fs::exists(current / "Base")) {
            return current;
        }
        if (current == current.root_path()) {
            break;
        }
        current = current.parent_path();
    }
    return fs::current_path();
}

ProjectPaths makeProjectPaths() {
    const fs::path project_root = findProjectRoot();
    const fs::path kitti_dir = project_root / "Kitti";
    const fs::path base_dir = project_root / "Base";
    return {
        (kitti_dir / "camera-info.txt").string(),
        (kitti_dir / "left" / "%06d.png").string(),
        (kitti_dir / "gt-tum07.txt").string(),
        (base_dir / "position.txt").string(),
    };
}

int clampFrameCount(int requested_count, int available_count) {
    if (requested_count > 0) {
        return min(requested_count, available_count);
    }
    return available_count;
}

bool canConnectUnixSocket(const fs::path& socket_path) {
    if (!fs::exists(socket_path)) {
        return false;
    }

    const string socket_text = socket_path.string();
    if (socket_text.size() >= sizeof(sockaddr_un::sun_path)) {
        return false;
    }

    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        return false;
    }

    sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", socket_text.c_str());
    const int result = ::connect(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    const int saved_errno = errno;
    ::close(fd);

    return result == 0 || saved_errno == EISCONN;
}

bool isAccessibleX11Display(const char* display) {
    if (display == nullptr || display[0] == '\0') {
        return false;
    }

    const string display_name(display);
    const size_t colon_pos = display_name.rfind(':');
    if (colon_pos == string::npos) {
        return false;
    }

    size_t number_start = colon_pos + 1;
    size_t number_end = number_start;
    while (number_end < display_name.size() &&
           std::isdigit(static_cast<unsigned char>(display_name[number_end]))) {
        ++number_end;
    }
    if (number_start == number_end) {
        return false;
    }

    const string display_number = display_name.substr(number_start, number_end - number_start);
    return canConnectUnixSocket(fs::path("/tmp/.X11-unix") / ("X" + display_number));
}

bool isAccessibleWaylandDisplay(const char* wayland_display) {
    if (wayland_display == nullptr || wayland_display[0] == '\0') {
        return false;
    }

    const char* runtime_dir = std::getenv("XDG_RUNTIME_DIR");
    if (runtime_dir == nullptr || runtime_dir[0] == '\0') {
        return false;
    }

    return canConnectUnixSocket(fs::path(runtime_dir) / wayland_display);
}

Mat readAndResizeImage(const string& image_pattern, int frame_id, double image_scale) {
    Mat image = readImageOrEmpty(image_pattern, frame_id);
    if (!image.empty() && image_scale != 1.0) {
        Mat resized;
        resize(image, resized, Size(), image_scale, image_scale, INTER_LINEAR);
        return resized;
    }
    return image;
}

Matx33d toMatx33d(const Mat& rotation) {
    return Matx33d(
        rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
        rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
        rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));
}

Vec3d toVec3d(const Mat& translation) {
    return Vec3d(
        translation.at<double>(0, 0),
        translation.at<double>(1, 0),
        translation.at<double>(2, 0));
}

Point2f projectTracePoint(
    double x,
    double z,
    double min_x,
    double min_z,
    double scale,
    double offset_x,
    double offset_y) {
    return Point2f(
        static_cast<float>(offset_x + (x - min_x) * scale),
        static_cast<float>(kTraceCanvasSize - offset_y - (z - min_z) * scale));
}

bool shouldReportProgress(int frame_id, int max_frames) {
    return frame_id < 5 || frame_id % kProgressReportInterval == 0 || frame_id + 1 == max_frames;
}

Mat renderTraceCanvas(
    const vector<Vec3d>& gt_positions,
    const vector<Vec3d>& estimated_positions,
    const vector<int>& frame_ids,
    int current_frame,
    int max_frames) {
    Mat trace(kTraceCanvasSize, kTraceCanvasSize, CV_8UC3, Scalar(255, 255, 255));
    putText(trace, "Black--Ground Truth", Point2f(10, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1, 8);
    putText(trace, "Green--VO", Point2f(10, 50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);

    if (estimated_positions.empty() || frame_ids.empty()) {
        return trace;
    }

    double min_x = numeric_limits<double>::max();
    double max_x = numeric_limits<double>::lowest();
    double min_z = numeric_limits<double>::max();
    double max_z = numeric_limits<double>::lowest();

    for (size_t i = 0; i < estimated_positions.size(); ++i) {
        const int frame_id = frame_ids[i];
        const Vec3d& gt = gt_positions[frame_id];
        const Vec3d& est = estimated_positions[i];
        min_x = min(min_x, min(gt[0], est[0]));
        max_x = max(max_x, max(gt[0], est[0]));
        min_z = min(min_z, min(gt[2], est[2]));
        max_z = max(max_z, max(gt[2], est[2]));
    }

    const double range_x = max(1.0, max_x - min_x);
    const double range_z = max(1.0, max_z - min_z);
    const double drawable = static_cast<double>(kTraceCanvasSize) - 2.0 * kTracePadding;
    const double scale = min(drawable / range_x, drawable / range_z);
    const double used_width = range_x * scale;
    const double used_height = range_z * scale;
    const double offset_x = (static_cast<double>(kTraceCanvasSize) - used_width) * 0.5;
    const double offset_y = (static_cast<double>(kTraceCanvasSize) - used_height) * 0.5;

    for (size_t i = 0; i < estimated_positions.size(); ++i) {
        const int frame_id = frame_ids[i];
        const Vec3d& gt = gt_positions[frame_id];
        const Vec3d& est = estimated_positions[i];

        circle(trace, projectTracePoint(gt[0], gt[2], min_x, min_z, scale, offset_x, offset_y), 1, Scalar(0, 0, 0), 1);
        circle(trace, projectTracePoint(est[0], est[2], min_x, min_z, scale, offset_x, offset_y), 1, Scalar(0, 255, 0), 1);
    }

    char info[256];
    std::snprintf(
        info,
        sizeof(info),
        "Frames: %d/%d  X:[%.1f, %.1f]  Z:[%.1f, %.1f]",
        current_frame + 1,
        max_frames,
        min_x,
        max_x,
        min_z,
        max_z);
    putText(trace, info, Point2f(10, static_cast<float>(kTraceCanvasSize - 15)), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(80, 80, 80), 1, 8);

    return trace;
}

}  // namespace

bool loadCameraIntrinsics(const string& camera_info_path, CameraIntrinsics& intrinsics) {
    ifstream input(camera_info_path);
    if (!input.is_open()) {
        cerr << "Unable to open camera info: " << camera_info_path << endl;
        return false;
    }

    string line;
    while (getline(input, line)) {
        if (line.rfind("Camera.fx:", 0) == 0) {
            intrinsics.fx = stod(line.substr(line.find(':') + 1));
        } else if (line.rfind("Camera.fy:", 0) == 0) {
            intrinsics.fy = stod(line.substr(line.find(':') + 1));
        } else if (line.rfind("Camera.cx:", 0) == 0) {
            intrinsics.cx = stod(line.substr(line.find(':') + 1));
        } else if (line.rfind("Camera.cy:", 0) == 0) {
            intrinsics.cy = stod(line.substr(line.find(':') + 1));
        }
    }

    return intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
}

vector<Vec3d> loadGroundTruthPositions(const string& pose_path) {
    ifstream input(pose_path);
    vector<Vec3d> positions;
    if (!input.is_open()) {
        cerr << "Unable to open pose file: " << pose_path << endl;
        return positions;
    }

    string line;
    while (getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        istringstream stream(line);
        double timestamp = 0.0;
        double tx = 0.0;
        double ty = 0.0;
        double tz = 0.0;
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        if (stream >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
            positions.emplace_back(tx, ty, tz);
        }
    }

    return positions;
}

Mat buildCameraMatrix(const CameraIntrinsics& intrinsics) {
    return (Mat_<double>(3, 3) <<
        intrinsics.fx, 0.0, intrinsics.cx,
        0.0, intrinsics.fy, intrinsics.cy,
        0.0, 0.0, 1.0);
}

void scaleCameraIntrinsics(CameraIntrinsics& intrinsics, double scale) {
    intrinsics.fx *= scale;
    intrinsics.fy *= scale;
    intrinsics.cx *= scale;
    intrinsics.cy *= scale;
}

string formatImagePath(const string& image_pattern, int frame_id) {
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), image_pattern.c_str(), frame_id);
    return string(buffer);
}

Mat readImageOrEmpty(const string& image_pattern, int frame_id) {
    return imread(formatImagePath(image_pattern, frame_id), IMREAD_GRAYSCALE);
}

void featureDetection(const Mat& image, vector<Point2f>& points, int max_corners) {
    static Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(20, true);
    vector<KeyPoint> keypoints;
    detector->detect(image, keypoints);
    if (static_cast<int>(keypoints.size()) > max_corners) {
        nth_element(
            keypoints.begin(),
            keypoints.begin() + max_corners,
            keypoints.end(),
            [](const KeyPoint& lhs, const KeyPoint& rhs) { return lhs.response > rhs.response; });
        keypoints.resize(max_corners);
    }
    KeyPoint::convert(keypoints, points);
    if (!points.empty()) {
        cornerSubPix(
            image,
            points,
            Size(10, 10),
            Size(-1, -1),
            TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03));
    }
}

int featureTracking(
    const Mat& prev_image,
    const Mat& curr_image,
    vector<Point2f>& prev_points,
    vector<Point2f>& curr_points,
    vector<uchar>& status) {
    vector<float> errors;
    vector<Point2f> backward_points;
    vector<uchar> backward_status;
    vector<float> backward_errors;
    static const Size kWindowSize(21, 21);
    static const TermCriteria kTermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
    calcOpticalFlowPyrLK(
        prev_image,
        curr_image,
        prev_points,
        curr_points,
        status,
        errors,
        kWindowSize,
        3,
        kTermCriteria);
    calcOpticalFlowPyrLK(
        curr_image,
        prev_image,
        curr_points,
        backward_points,
        backward_status,
        backward_errors,
        kWindowSize,
        3,
        kTermCriteria);

    vector<Point2f> filtered_prev;
    vector<Point2f> filtered_curr;
    filtered_prev.reserve(prev_points.size());
    filtered_curr.reserve(curr_points.size());

    const int width = curr_image.cols;
    const int height = curr_image.rows;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i] == 0 || backward_status[i] == 0) {
            continue;
        }
        const Point2f& point = curr_points[i];
        if (point.x < 0.0f || point.y < 0.0f || point.x >= width || point.y >= height) {
            continue;
        }
        if (norm(prev_points[i] - backward_points[i]) > kMaxForwardBackwardError) {
            continue;
        }
        filtered_prev.push_back(prev_points[i]);
        filtered_curr.push_back(point);
    }

    prev_points.swap(filtered_prev);
    curr_points.swap(filtered_curr);
    return static_cast<int>(curr_points.size());
}

void compactPointsByMask(vector<Point2f>& prev_points, vector<Point2f>& curr_points, const Mat& mask) {
    vector<Point2f> filtered_prev;
    vector<Point2f> filtered_curr;
    filtered_prev.reserve(prev_points.size());
    filtered_curr.reserve(curr_points.size());

    for (int i = 0; i < mask.rows && i < static_cast<int>(prev_points.size()) && i < static_cast<int>(curr_points.size()); ++i) {
        const uchar keep = mask.rows == 1 ? mask.at<uchar>(0, i) : mask.at<uchar>(i, 0);
        if (keep == 0) {
            continue;
        }
        filtered_prev.push_back(prev_points[i]);
        filtered_curr.push_back(curr_points[i]);
    }

    prev_points.swap(filtered_prev);
    curr_points.swap(filtered_curr);
}

Point2f pixel2cam(const Point2d& pixel, const Mat& camera_matrix) {
    return Point2f(
        static_cast<float>((pixel.x - camera_matrix.at<double>(0, 2)) / camera_matrix.at<double>(0, 0)),
        static_cast<float>((pixel.y - camera_matrix.at<double>(1, 2)) / camera_matrix.at<double>(1, 1)));
}

void triangulation(
    const vector<Point2f>& points_1,
    const vector<Point2f>& points_2,
    const Mat& camera_matrix,
    const Mat& rotation,
    const Mat& translation,
    vector<Point3f>& points) {
    Mat pose_1 = (Mat_<float>(3, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    Mat pose_2 = (Mat_<float>(3, 4) <<
        rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), translation.at<double>(0, 0),
        rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), translation.at<double>(1, 0),
        rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), translation.at<double>(2, 0));

    vector<Point2f> normalized_1;
    vector<Point2f> normalized_2;
    normalized_1.reserve(points_1.size());
    normalized_2.reserve(points_2.size());

    for (size_t i = 0; i < points_1.size(); ++i) {
        normalized_1.push_back(pixel2cam(points_1[i], camera_matrix));
        normalized_2.push_back(pixel2cam(points_2[i], camera_matrix));
    }

    Mat points_4d;
    triangulatePoints(pose_1, pose_2, normalized_1, normalized_2, points_4d);

    points.clear();
    for (int i = 0; i < points_4d.cols; ++i) {
        Mat column = points_4d.col(i);
        column /= column.at<float>(3, 0);
        points.emplace_back(
            column.at<float>(0, 0),
            column.at<float>(1, 0),
            column.at<float>(2, 0));
    }
}

double getAbsoluteScale(const vector<Vec3d>& gt_positions, int frame_id) {
    if (frame_id <= 0 || frame_id >= static_cast<int>(gt_positions.size())) {
        return 0.0;
    }

    const Vec3d& prev = gt_positions[frame_id - 1];
    const Vec3d& curr = gt_positions[frame_id];
    const Vec3d delta = curr - prev;
    return sqrt(delta.dot(delta));
}

void appendPose(ofstream& output, const Vec3d& translation) {
    output << translation[0] << ' '
           << translation[1] << ' '
           << translation[2] << '\n';
}

bool isVisualizationEnabled() {
    const char* explicit_gui = std::getenv("VO_ENABLE_GUI");
    if (explicit_gui != nullptr && std::string(explicit_gui) == "0") {
        return false;
    }

    const char* display = std::getenv("DISPLAY");
    const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
    return isAccessibleX11Display(display) || isAccessibleWaylandDisplay(wayland_display);
}

bool isVerboseLoggingEnabled() {
    const char* verbose = std::getenv("VO_VERBOSE_LOG");
    return verbose != nullptr && std::string(verbose) == "1";
}

double getImageScale() {
    const char* scale_text = std::getenv("VO_IMAGE_SCALE");
    if (scale_text == nullptr) {
        return kDefaultImageScale;
    }

    const double scale = std::atof(scale_text);
    if (scale <= 0.0 || scale > 1.0) {
        return kDefaultImageScale;
    }
    return scale;
}

int main(int argc, char** argv) {
    const ProjectPaths paths = makeProjectPaths();
    CameraIntrinsics intrinsics;
    if (!loadCameraIntrinsics(paths.camera_info_path, intrinsics)) {
        return -1;
    }

    const double image_scale = getImageScale();
    scaleCameraIntrinsics(intrinsics, image_scale);
    const Mat camera_matrix = buildCameraMatrix(intrinsics);
    const vector<Vec3d> gt_positions = loadGroundTruthPositions(paths.gt_pose_path);
    if (gt_positions.size() < 2) {
        cerr << "Ground truth pose count is insufficient." << endl;
        return -1;
    }

    const int available_frames = static_cast<int>(gt_positions.size());
    const int requested_frames = argc > 1 ? atoi(argv[1]) : available_frames;
    const int max_frames = clampFrameCount(requested_frames, available_frames);
    if (max_frames < 2) {
        cerr << "Need at least two frames." << endl;
        return -1;
    }

    Mat img_1 = readAndResizeImage(paths.left_image_pattern, 0, image_scale);
    Mat img_2 = readAndResizeImage(paths.left_image_pattern, 1, image_scale);
    if (img_1.empty() || img_2.empty()) {
        cerr << "Error reading initial images." << endl;
        return -1;
    }

    vector<Point2f> points1;
    vector<Point2f> points2;
    featureDetection(img_1, points1, kTargetFeatureCount);
    points2.reserve(points1.size());

    vector<uchar> status;
    featureTracking(img_1, img_2, points1, points2, status);
    if (points1.size() < 8 || points2.size() < 8) {
        cerr << "Not enough tracked features after initialization." << endl;
        return -1;
    }

    const double focal_length = intrinsics.fx;
    const Point2d principal_point(intrinsics.cx, intrinsics.cy);

    Mat essential_matrix;
    Mat rotation;
    Mat translation;
    Mat mask;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point, RANSAC, 0.999, 1.0, mask);
    const int initial_inliers =
        recoverPose(essential_matrix, points1, points2, rotation, translation, focal_length, principal_point, mask);
    if (initial_inliers < kMinInitialPoseInliers) {
        cerr << "Not enough pose inliers after initialization: "
             << initial_inliers << " inliers from " << points1.size() << " tracked features." << endl;
        return -1;
    }
    compactPointsByMask(points1, points2, mask);

    const bool enable_visualization = isVisualizationEnabled();
    const bool verbose_logging = isVerboseLoggingEnabled();
    if (enable_visualization) {
        namedWindow("CAMERA IMAGE", WINDOW_AUTOSIZE);
        namedWindow("Trajectory Drawing", WINDOW_AUTOSIZE);
    } else {
        cout << "Visualization disabled. GUI opens by default when a usable local display socket is present. "
             << "Set VO_ENABLE_GUI=0 to force headless mode." << endl;
    }
    if (requested_frames > 0 && requested_frames < available_frames) {
        cout << "Frame cap enabled: " << max_frames << "/" << available_frames << endl;
    } else {
        cout << "Using all available frames: " << available_frames << endl;
    }
    cout << "Processing " << max_frames << " frames..." << endl;
    Mat prev_image = img_2;
    Mat curr_image;
    vector<Point2f> prev_features = points2;
    vector<Point2f> curr_features;
    curr_features.reserve(prev_features.size());

    PoseRecord raw_pose;
    raw_pose.rotation = toMatx33d(rotation.t());
    raw_pose.translation = toVec3d(translation);

    vector<Vec3d> trajectory;
    trajectory.reserve(max_frames);
    vector<int> trajectory_frame_ids;
    trajectory_frame_ids.reserve(max_frames);
    trajectory.push_back(raw_pose.translation);
    trajectory_frame_ids.push_back(1);

    future<Mat> next_frame_future;
    if (max_frames > 2) {
        next_frame_future = async(std::launch::async, readAndResizeImage, paths.left_image_pattern, 2, image_scale);
    }

    for (int frame_id = 2; frame_id < max_frames; ++frame_id) {
        curr_image = next_frame_future.valid()
            ? next_frame_future.get()
            : readAndResizeImage(paths.left_image_pattern, frame_id, image_scale);
        if (frame_id + 1 < max_frames) {
            next_frame_future = async(std::launch::async, readAndResizeImage, paths.left_image_pattern, frame_id + 1, image_scale);
        }
        if (curr_image.empty()) {
            cerr << "Skipping missing frame: " << formatImagePath(paths.left_image_pattern, frame_id) << endl;
            continue;
        }

        vector<uchar> tracking_status;
        curr_features.clear();
        featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
        if (prev_features.size() < 8 || curr_features.size() < 8) {
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": not enough tracked features, re-detecting." << endl;
            }
            featureDetection(prev_image, prev_features, kTargetFeatureCount);
            curr_features.clear();
            featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
            if (prev_features.size() < 8 || curr_features.size() < 8) {
                prev_image = curr_image;
                prev_features = curr_features;
                if (shouldReportProgress(frame_id, max_frames)) {
                    cout << "Frame " << frame_id << "/" << (max_frames - 1)
                         << " tracked=0 status=reinit-failed" << endl;
                }
                continue;
            }
        }

        essential_matrix = findEssentialMat(
            prev_features,
            curr_features,
            focal_length,
            principal_point,
            RANSAC,
            0.999,
            1.0,
            mask);
        const int inlier_count = recoverPose(
            essential_matrix,
            prev_features,
            curr_features,
            rotation,
            translation,
            focal_length,
            principal_point,
            mask);
        if (inlier_count < kMinPoseInliers) {
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": pose inliers dropped to " << inlier_count << ", skipping update." << endl;
            }
            featureDetection(curr_image, curr_features, kTargetFeatureCount);
            trajectory.push_back(raw_pose.translation);
            trajectory_frame_ids.push_back(frame_id);
            prev_image = curr_image;
            prev_features = curr_features;
            if (shouldReportProgress(frame_id, max_frames)) {
                cout << "Frame " << frame_id << "/" << (max_frames - 1)
                     << " tracked=" << curr_features.size()
                     << " inliers=" << inlier_count
                     << " status=skip-pose" << endl;
            }
            continue;
        }

        const Vec3d delta_translation = toVec3d(translation);

        const double scale = getAbsoluteScale(gt_positions, frame_id);
        if (scale > kMinTranslationScale) {
            raw_pose.translation += scale * (raw_pose.rotation * (-delta_translation));
            raw_pose.rotation = toMatx33d(rotation.t()) * raw_pose.rotation;
        } else if (verbose_logging) {
            cout << "Frame " << frame_id << ": scale below threshold." << endl;
        }

        trajectory.push_back(raw_pose.translation);
        trajectory_frame_ids.push_back(frame_id);
        compactPointsByMask(prev_features, curr_features, mask);

        if (curr_features.size() < kMinTrackedFeatures) {
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": tracked features down to " << curr_features.size() << ", re-detecting." << endl;
            }
            featureDetection(curr_image, curr_features, kTargetFeatureCount);
        }

        if (shouldReportProgress(frame_id, max_frames)) {
            cout << "Frame " << frame_id << "/" << (max_frames - 1)
                 << " tracked=" << curr_features.size()
                 << " inliers=" << inlier_count
                 << " pose=(" << raw_pose.translation[0] << ", "
                 << raw_pose.translation[1] << ", "
                 << raw_pose.translation[2] << ")" << endl;
        }

        prev_image = curr_image;
        prev_features = curr_features;

        if (enable_visualization) {
            Mat trace = renderTraceCanvas(gt_positions, trajectory, trajectory_frame_ids, frame_id, max_frames);
            imshow("CAMERA IMAGE", curr_image);
            imshow("Trajectory Drawing", trace);
            waitKey(1);
        }
    }

    ofstream output(paths.output_pose_path);
    if (!output.is_open()) {
        cerr << "Unable to open output file: " << paths.output_pose_path << endl;
        return -1;
    }
    for (const Vec3d& position : trajectory) {
        appendPose(output, position);
    }
    output.close();
    Mat trace = renderTraceCanvas(gt_positions, trajectory, trajectory_frame_ids, max_frames - 1, max_frames);
    imwrite("map2.png", trace);
    cout << "Finished. Saved trajectory to " << paths.output_pose_path << " and map to map2.png" << endl;
    return 0;
}
