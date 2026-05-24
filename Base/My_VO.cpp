#include "My_VO.h"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

namespace {

constexpr int kMinNumFeat = 1000;
constexpr double kMinTranslationScale = 0.1;

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

Mat makeTraceCanvas() {
    Mat trace = Mat::zeros(800, 800, CV_8UC3);
    trace.setTo(Scalar(255, 255, 255));
    putText(trace, "Black--Ground Truth", Point2f(10, 50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1, 8);
    putText(trace, "Green--VO", Point2f(10, 70), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);
    return trace;
}

int clampFrameCount(int requested_count, int available_count) {
    if (requested_count > 0) {
        return min(requested_count, available_count);
    }
    return available_count;
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

string formatImagePath(const string& image_pattern, int frame_id) {
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), image_pattern.c_str(), frame_id);
    return string(buffer);
}

Mat readImageOrEmpty(const string& image_pattern, int frame_id) {
    return imread(formatImagePath(image_pattern, frame_id), IMREAD_GRAYSCALE);
}

void featureDetection(const Mat& image, vector<Point2f>& points) {
    vector<KeyPoint> keypoints;
    Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(10);
    detector->detect(image, keypoints);
    KeyPoint::convert(keypoints, points, vector<int>());
}

void featureTracking(
    const Mat& prev_image,
    const Mat& curr_image,
    vector<Point2f>& prev_points,
    vector<Point2f>& curr_points,
    vector<uchar>& status) {
    vector<float> errors;
    calcOpticalFlowPyrLK(prev_image, curr_image, prev_points, curr_points, status, errors);

    int removed = 0;
    for (int i = 0; i < static_cast<int>(status.size()); ++i) {
        const Point2f& point = curr_points.at(i - removed);
        if (status.at(i) == 0 || point.x < 0 || point.y < 0) {
            prev_points.erase(prev_points.begin() + (i - removed));
            curr_points.erase(curr_points.begin() + (i - removed));
            ++removed;
        }
    }
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

bool isForwardMotion(const Mat& translation) {
    return -translation.at<double>(2, 0) > -translation.at<double>(0, 0) &&
           -translation.at<double>(2, 0) > -translation.at<double>(1, 0);
}

void appendPose(ofstream& output, const Mat& translation) {
    output << translation.at<double>(0, 0) << ' '
           << translation.at<double>(1, 0) << ' '
           << translation.at<double>(2, 0) << '\n';
}

bool isVisualizationEnabled() {
    const char* explicit_gui = std::getenv("VO_ENABLE_GUI");
    if (explicit_gui == nullptr || std::string(explicit_gui) != "1") {
        return false;
    }
    const char* display = std::getenv("DISPLAY");
    const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
    return (display != nullptr && display[0] != '\0') ||
           (wayland_display != nullptr && wayland_display[0] != '\0');
}

int main(int argc, char** argv) {
    const ProjectPaths paths = makeProjectPaths();
    CameraIntrinsics intrinsics;
    if (!loadCameraIntrinsics(paths.camera_info_path, intrinsics)) {
        return -1;
    }

    const Mat camera_matrix = buildCameraMatrix(intrinsics);
    const vector<Vec3d> gt_positions = loadGroundTruthPositions(paths.gt_pose_path);
    if (gt_positions.size() < 2) {
        cerr << "Ground truth pose count is insufficient." << endl;
        return -1;
    }

    const int available_frames = static_cast<int>(gt_positions.size());
    const int max_frames = clampFrameCount(argc > 1 ? atoi(argv[1]) : available_frames, available_frames);
    if (max_frames < 2) {
        cerr << "Need at least two frames." << endl;
        return -1;
    }

    Mat img_1 = readImageOrEmpty(paths.left_image_pattern, 0);
    Mat img_2 = readImageOrEmpty(paths.left_image_pattern, 1);
    if (img_1.empty() || img_2.empty()) {
        cerr << "Error reading initial images." << endl;
        return -1;
    }

    ofstream output(paths.output_pose_path);
    if (!output.is_open()) {
        cerr << "Unable to open output file: " << paths.output_pose_path << endl;
        return -1;
    }

    vector<Point2f> points1;
    vector<Point2f> points2;
    featureDetection(img_1, points1);

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
    recoverPose(essential_matrix, points1, points2, rotation, translation, focal_length, principal_point, mask);

    const bool enable_visualization = isVisualizationEnabled();
    if (enable_visualization) {
        namedWindow("CAMERA IMAGE", WINDOW_AUTOSIZE);
        namedWindow("Trajectory Drawing", WINDOW_AUTOSIZE);
    } else {
        cout << "Visualization disabled. Set VO_ENABLE_GUI=1 to enable windows in a local desktop session." << endl;
    }
    Mat trace = makeTraceCanvas();

    Mat prev_image = img_2;
    Mat curr_image;
    vector<Point2f> prev_features = points2;
    vector<Point2f> curr_features;

    PoseRecord raw_pose{rotation.clone(), translation.clone()};

    appendPose(output, raw_pose.translation);

    for (int frame_id = 2; frame_id < max_frames; ++frame_id) {
        curr_image = readImageOrEmpty(paths.left_image_pattern, frame_id);
        if (curr_image.empty()) {
            cerr << "Skipping missing frame: " << formatImagePath(paths.left_image_pattern, frame_id) << endl;
            continue;
        }

        vector<uchar> tracking_status;
        featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
        if (prev_features.size() < 8 || curr_features.size() < 8) {
            cerr << "Not enough tracked features at frame " << frame_id << ", re-detecting." << endl;
            featureDetection(prev_image, prev_features);
            featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
            if (prev_features.size() < 8 || curr_features.size() < 8) {
                prev_image = curr_image.clone();
                prev_features = curr_features;
                continue;
            }
        }

        essential_matrix = findEssentialMat(prev_features, curr_features, focal_length, principal_point, RANSAC, 0.999, 1.0, mask);
        recoverPose(essential_matrix, prev_features, curr_features, rotation, translation, focal_length, principal_point, mask);

        const double scale = getAbsoluteScale(gt_positions, frame_id);
        if (scale > kMinTranslationScale && isForwardMotion(translation)) {
            raw_pose.translation = raw_pose.translation + scale * (raw_pose.rotation * (-translation));
            raw_pose.rotation = rotation.inv() * raw_pose.rotation;
        } else {
            cout << "Frame " << frame_id << ": scale below threshold or translation is not forward-dominant." << endl;
        }

        const Point2f gt_trace(static_cast<float>(gt_positions[frame_id][0]) + 400.0f, static_cast<float>(gt_positions[frame_id][2]) + 150.0f);
        const Point2f raw_trace(static_cast<float>(raw_pose.translation.at<double>(0, 0)) + 400.0f, static_cast<float>(raw_pose.translation.at<double>(2, 0)) + 150.0f);
        circle(trace, gt_trace, 1, Scalar(0, 0, 0), 1);
        circle(trace, raw_trace, 1, Scalar(0, 255, 0), 1);

        appendPose(output, raw_pose.translation);

        if (curr_features.size() < kMinNumFeat) {
            cout << "Frame " << frame_id << ": tracked features down to " << curr_features.size() << ", re-detecting." << endl;
            featureDetection(curr_image, curr_features);
        }

        prev_image = curr_image.clone();
        prev_features = curr_features;

        if (enable_visualization) {
            imshow("CAMERA IMAGE", curr_image);
            imshow("Trajectory Drawing", trace);
            waitKey(1);
        }
    }

    output.close();
    imwrite("map2.png", trace);
    return 0;
}
