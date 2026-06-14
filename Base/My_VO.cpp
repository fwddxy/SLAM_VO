#include "My_VO.h"

#include <cmath>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

using namespace cv;
using namespace std;

namespace {

// VO 运行参数集中放在匿名命名空间内，避免污染全局符号。
// 这些阈值决定了特征数量、位姿估计可靠性、轨迹显示尺寸等行为：
// - kTargetFeatureCount：每次重新检测时最多保留的 FAST 特征点数量。
// - kMinTrackedFeatures：跟踪特征低于该数量时触发重新检测。
// - kMinTranslationScale：真值相邻帧距离过小时不更新平移，避免静止或抖动帧放大噪声。
// - kMaxForwardBackwardError：光流前后向一致性检查阈值，越小越严格。
// - kMinInitialPoseInliers/kMinPoseInliers：recoverPose 内点数量下限，用来过滤退化位姿。
constexpr int kTargetFeatureCount = 2000;
constexpr int kMinTrackedFeatures = 1000;
constexpr double kMinTranslationScale = 0.1;
constexpr double kDefaultImageScale = 1.0;
constexpr int kTraceCanvasSize = 1000;
constexpr int kProgressReportInterval = 50;
constexpr double kTracePadding = 40.0;
constexpr float kMaxForwardBackwardError = 1.0f;
constexpr int kMinInitialPoseInliers = 5;
constexpr int kMinPoseInliers = 20;
constexpr int kMinPnPInliers = 12;
constexpr int kMinTriangulatedLandmarks = 4;
constexpr float kPnPReprojectionError = 4.0f;
constexpr double kTriangulationReprojectionError = 5.0;
constexpr double kMinTriangulationParallaxPixels = 5.0;
constexpr double kMinTriangulationBaseline = 0.1;
constexpr double kMinLandmarkDepth = 0.1;
constexpr double kMaxLandmarkDepth = 80.0;
constexpr int kMinKeyframePnPInliers = 30;
constexpr int kPreferredKeyframePnPInliers = 80;
constexpr int kMaxKeyframeTrackAge = 20;
constexpr double kMaxPnPRotationDeltaDegrees = 8.0;
constexpr double kMaxPnPTranslationDeltaScale = 3.0;
constexpr double kTrustedPnPRotationDeltaDegrees = 1.5;
constexpr double kTrustedPnPTranslationDeltaScale = 0.75;
constexpr double kMaxOutputPnPScale = 1.0;
constexpr int kMinOutputPnPKeyframeAge = 1;
constexpr double kMinOutputPnPRotationDegrees = 0.0;
constexpr int kMaxOutputPnPTrackedFeatures = 2200;

namespace fs = std::filesystem;

struct RunOptions {
    // 运行区间控制参数。
    // start_frame/end_frame 用于选择原始序列中的闭区间；
    // frame_count 是从 start_frame 开始处理的帧数；
    // output_root_override 允许把结果输出到自定义目录。
    int start_frame = 0;
    int end_frame = -1;
    int frame_count = -1;
    std::string output_root_override;
};

fs::path findProjectRoot() {
    // 从当前目录逐级向上查找，直到同时找到 Kitti 和 Base 目录。
    // 这样程序既可以从工程根目录运行，也可以从 build/ 目录运行。
    // 如果始终没有找到工程根目录，则退回 current_path，让后续文件打开逻辑给出明确错误。
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

ProjectPaths makeProjectPaths(const fs::path& output_root_override = fs::path()) {
    // 根据工程根目录统一生成输入输出文件路径，避免依赖固定启动目录。
    // 输入：
    // - Kitti/camera-info.txt：相机内参配置。
    // - Kitti/left/%06d.png：左目灰度图序列，frame_id 会格式化为 000000.png。
    // - Kitti/gt-tum07.txt：TUM 格式真值轨迹。
    // 输出：
    // - output/full_run/position/position.txt：本程序估计出的 TUM 格式轨迹。
    // - output/full_run/traj/map2.png：本程序绘制的轨迹对比图。
    const fs::path project_root = findProjectRoot();
    const fs::path kitti_dir = project_root / "Kitti";
    const fs::path output_dir = output_root_override.empty()
        ? project_root / "output" / "full_run"
        : (output_root_override.is_absolute() ? output_root_override : project_root / output_root_override);
    return {
        project_root.string(),
        (kitti_dir / "camera-info.txt").string(),
        (kitti_dir / "left" / "%06d.png").string(),
        (kitti_dir / "gt-tum07.txt").string(),
        (output_dir / "position" / "position.txt").string(),
        (output_dir / "position" / "geometry_position.txt").string(),
        (output_dir / "traj" / "map2.png").string(),
    };
}

bool ensureParentDirectory(const string& output_path) {
    // 输出目录不存在时自动创建，保持源码目录和运行产物分离。
    const fs::path parent = fs::path(output_path).parent_path();
    if (parent.empty()) {
        return true;
    }
    std::error_code error;
    fs::create_directories(parent, error);
    if (error) {
        cerr << "Unable to create output directory: " << parent << " (" << error.message() << ")" << endl;
        return false;
    }
    return true;
}

int clampFrameCount(int requested_count, int available_count) {
    // 命令行传入正数时限制处理帧数，否则默认使用全部可用真值帧。
    // 这里同时保证最大处理帧数不会超过真值轨迹数量，避免后续按 frame_id 访问真值时越界。
    if (requested_count > 0) {
        return min(requested_count, available_count);
    }
    return available_count;
}

bool canConnectUnixSocket(const fs::path& socket_path) {
    // 通过实际连接 Unix socket 判断 GUI 显示服务是否可用。
    // 只检查环境变量是否存在并不可靠：例如 SSH、容器或无头环境可能设置了 DISPLAY，
    // 但没有可连接的 X11/Wayland socket。这里真实 connect 一次，避免 imshow 后程序卡住或报错。
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
    // DISPLAY 通常形如 :0 或 hostname:0.0，这里解析显示编号后检查 /tmp/.X11-unix/X*。
    // 例如 DISPLAY=:1 时，对应的本地 socket 是 /tmp/.X11-unix/X1。
    // 若 DISPLAY 中没有冒号或冒号后没有数字，则无法判断显示编号，直接认为不可用。
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
    // Wayland socket 位于 XDG_RUNTIME_DIR 下，存在且可连接才认为可以显示窗口。
    // 常见值如 WAYLAND_DISPLAY=wayland-0，对应 $XDG_RUNTIME_DIR/wayland-0。
    // 两个环境变量缺任意一个，都无法定位 Wayland socket。
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
    // 支持通过 VO_IMAGE_SCALE 缩小图像，图像缩放后相机内参也会同步缩放。
    // 缩小图像可以明显减少 FAST 和 LK 光流的计算量；INTER_LINEAR 适合普通灰度图缩放。
    // 如果读取失败，image 为空，此处不会 resize，直接把空 Mat 返回给调用者判断。
    Mat image = readImageOrEmpty(image_pattern, frame_id);
    if (!image.empty() && image_scale != 1.0) {
        Mat resized;
        resize(image, resized, Size(), image_scale, image_scale, INTER_LINEAR);
        return resized;
    }
    return image;
}

Matx33d toMatx33d(const Mat& rotation) {
    // OpenCV 的 recoverPose 返回 Mat，这里转换为更轻量的固定尺寸矩阵方便累计位姿。
    // Matx33d 是栈上固定大小矩阵，和 Vec3d 做乘法更直接，也减少动态 Mat 的类型转换。
    return Matx33d(
        rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
        rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
        rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));
}

Vec3d toVec3d(const Mat& translation) {
    // 平移向量统一转成 Vec3d，后续直接做向量加法和尺度乘法。
    // recoverPose 返回的 translation 是 3x1 Mat，数值只有方向，真实长度需要另行恢复尺度。
    return Vec3d(
        translation.at<double>(0, 0),
        translation.at<double>(1, 0),
        translation.at<double>(2, 0));
}

void poseToWorldToCamera(const PoseRecord& pose, Matx33d& rotation_wc, Vec3d& translation_wc) {
    // PoseRecord 内部保存的是相机到世界的 R_cw 和相机中心 t_cw。
    // PnP、投影矩阵等 OpenCV 接口通常需要世界到相机的 R_wc 和 t_wc：
    //   X_cam = R_wc * X_world + t_wc
    // 因此 R_wc = R_cw^T，t_wc = -R_wc * t_cw。
    rotation_wc = pose.rotation.t();
    translation_wc = -(rotation_wc * pose.translation);
}

PoseRecord poseFromRelativeMotion(
    const PoseRecord& prev_pose,
    const Mat& relative_rotation,
    const Mat& relative_translation,
    double scale) {
    // 从上一帧几何位姿和 recoverPose 的相对运动推算当前几何位姿。
    // relative_rotation/relative_translation 表示上一帧到当前帧的相机运动；
    // translation 只有方向，真实长度由 scale 提供。
    PoseRecord curr_pose;
    curr_pose.rotation = prev_pose.rotation * toMatx33d(relative_rotation.t());
    curr_pose.translation = prev_pose.translation + curr_pose.rotation * (-scale * toVec3d(relative_translation));
    return curr_pose;
}

PoseRecord poseFromInitialRelativeMotion(
    const Mat& relative_rotation,
    const Mat& relative_translation,
    double scale) {
    // 初始两帧没有上一帧累计位姿，所以从单位位姿开始积分一次相对运动。
    return poseFromRelativeMotion(PoseRecord(), relative_rotation, relative_translation, scale);
}

PoseRecord accumulateOutputPose(
    const PoseRecord& prev_pose,
    const Mat& relative_rotation,
    const Mat& relative_translation,
    double scale) {
    // 输出轨迹沿用原始 2D-2D 累计方式，和 geometry_pose 的积分公式略有区别。
    // 保留这个函数是为了兼容原有轨迹方向和评估结果。
    PoseRecord curr_pose;
    curr_pose.translation = prev_pose.translation + scale * (prev_pose.rotation * (-toVec3d(relative_translation)));
    curr_pose.rotation = toMatx33d(relative_rotation.t()) * prev_pose.rotation;
    return curr_pose;
}

PoseRecord poseFromInitialOutputMotion(
    const Mat& relative_rotation,
    const Mat& relative_translation,
    double scale) {
    // 初始化输出轨迹的第一段相对运动。
    // 若真值尺度不可用，则至少保留 recoverPose 给出的单位方向，避免初始位置全为零。
    PoseRecord pose;
    pose.rotation = toMatx33d(relative_rotation.t());
    pose.translation = (scale > 0.0) ? scale * toVec3d(relative_translation) : toVec3d(relative_translation);
    return pose;
}

bool relativeMotionFromPoses(
    const PoseRecord& prev_pose,
    const PoseRecord& curr_pose,
    Mat& relative_rotation,
    Mat& relative_translation,
    double& relative_scale) {
    // 根据两个绝对位姿反推出 recoverPose 风格的相对运动。
    // 这个函数主要用于 PnP 结果被接受后，计算 PnP 位姿相对于上一几何位姿的运动量。
    const Matx33d relative_rotation_matx = curr_pose.rotation.t() * prev_pose.rotation;
    relative_rotation = (Mat_<double>(3, 3) <<
        relative_rotation_matx(0, 0), relative_rotation_matx(0, 1), relative_rotation_matx(0, 2),
        relative_rotation_matx(1, 0), relative_rotation_matx(1, 1), relative_rotation_matx(1, 2),
        relative_rotation_matx(2, 0), relative_rotation_matx(2, 1), relative_rotation_matx(2, 2));

    const Vec3d delta_world = curr_pose.translation - prev_pose.translation;
    const Vec3d scaled_relative_translation = -(curr_pose.rotation.t() * delta_world);
    relative_scale = norm(scaled_relative_translation);
    if (relative_scale <= 1e-9) {
        // 平移长度接近 0 时方向没有意义，返回 false 让调用方丢弃该相对运动。
        relative_translation = (Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
        return false;
    }

    const Vec3d direction = scaled_relative_translation / relative_scale;
    relative_translation = (Mat_<double>(3, 1) << direction[0], direction[1], direction[2]);
    return true;
}

PoseRecord poseFromPnP(const Mat& rvec, const Mat& tvec) {
    // solvePnP 输出的是世界到相机的 rvec/tvec。
    // 这里转换回 PoseRecord 的相机到世界表示，便于和 VO 累计位姿统一比较。
    Mat rotation_wc_mat;
    Rodrigues(rvec, rotation_wc_mat);

    PoseRecord pose;
    const Matx33d rotation_wc = toMatx33d(rotation_wc_mat);
    const Vec3d translation_wc = toVec3d(tvec);
    pose.rotation = rotation_wc.t();
    pose.translation = pose.rotation * (-translation_wc);
    return pose;
}

void poseToPnPGuess(const PoseRecord& pose, Mat& rvec, Mat& tvec) {
    // 将当前 VO 估计位姿转换为 solvePnP 可使用的初值。
    // useExtrinsicGuess=true 时，OpenCV 会从这个初值附近优化，降低跳到错误解的概率。
    Matx33d rotation_wc;
    Vec3d translation_wc;
    poseToWorldToCamera(pose, rotation_wc, translation_wc);

    Mat rotation_wc_mat(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation_wc_mat.at<double>(row, col) = rotation_wc(row, col);
        }
    }
    Rodrigues(rotation_wc_mat, rvec);
    tvec = (Mat_<double>(3, 1) << translation_wc[0], translation_wc[1], translation_wc[2]);
}

Mat buildProjectionMatrix(const Mat& camera_matrix, const PoseRecord& pose) {
    // 构造完整投影矩阵 P = K [R_wc | t_wc]，用于把世界点投影到像素平面。
    Matx33d rotation_wc;
    Vec3d translation_wc;
    poseToWorldToCamera(pose, rotation_wc, translation_wc);

    Mat extrinsic = (Mat_<double>(3, 4) <<
        rotation_wc(0, 0), rotation_wc(0, 1), rotation_wc(0, 2), translation_wc[0],
        rotation_wc(1, 0), rotation_wc(1, 1), rotation_wc(1, 2), translation_wc[1],
        rotation_wc(2, 0), rotation_wc(2, 1), rotation_wc(2, 2), translation_wc[2]);
    return camera_matrix * extrinsic;
}

Point2d projectWorldPoint(const Point3d& point_world, const Mat& camera_matrix, const PoseRecord& pose) {
    // 将一个世界坐标系下的 3D 点投影到当前相机图像中。
    // 返回 NaN 表示点在相机后方或深度无效，调用方会把该点过滤掉。
    Matx33d rotation_wc;
    Vec3d translation_wc;
    poseToWorldToCamera(pose, rotation_wc, translation_wc);

    const Vec3d point_camera = rotation_wc * Vec3d(point_world.x, point_world.y, point_world.z) + translation_wc;
    if (point_camera[2] <= 1e-9) {
        return Point2d(
            numeric_limits<double>::quiet_NaN(),
            numeric_limits<double>::quiet_NaN());
    }

    const double fx = camera_matrix.at<double>(0, 0);
    const double fy = camera_matrix.at<double>(1, 1);
    const double cx = camera_matrix.at<double>(0, 2);
    const double cy = camera_matrix.at<double>(1, 2);
    return Point2d(
        fx * point_camera[0] / point_camera[2] + cx,
        fy * point_camera[1] / point_camera[2] + cy);
}

double rotationAngleDegrees(const Matx33d& lhs, const Matx33d& rhs) {
    // 根据两个旋转矩阵的相对旋转迹计算夹角，结果单位为度。
    // clamp 用于抵消浮点误差，避免 acos 输入略微超过 [-1, 1]。
    const Matx33d delta = lhs * rhs.t();
    const double cosine = std::clamp((delta(0, 0) + delta(1, 1) + delta(2, 2) - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / CV_PI;
}

bool isPnPPoseConsistent(const PoseRecord& pnp_pose, const PoseRecord& reference_pose, double frame_scale) {
    // PnP 可能受到错误 3D-2D 对应影响而跳变，因此先和 2D-2D 主干位姿做一致性检查。
    // 平移误差阈值随当前帧真值尺度放大，同时设置 0.5m 下限避免小尺度帧过严。
    const double translation_limit = std::max(0.5, frame_scale * kMaxPnPTranslationDeltaScale);
    const double translation_delta = norm(pnp_pose.translation - reference_pose.translation);
    if (translation_delta > translation_limit) {
        return false;
    }
    return rotationAngleDegrees(pnp_pose.rotation, reference_pose.rotation) <= kMaxPnPRotationDeltaDegrees;
}

bool shouldApplyPnPToOutput(
    const PoseRecord& pnp_pose,
    const PoseRecord& reference_pose,
    double frame_scale,
    int pnp_inlier_count,
    int keyframe_map_age,
    int two_view_track_count,
    double two_view_rotation_degrees) {
    // 这里决定“PnP 结果是否真的写入最终输出轨迹”。
    // 当前策略非常保守：PnP 只作为 2D-2D 的补偿项，而不是主干位姿来源。
    // 只有在小尺度、PnP 内点很多、PnP 与 2D-2D 姿态差异很小、
    // 短时地图已经累计到足够年龄、当前 2D-2D 跟踪数已经明显下降、
    // 并且当前相机运动是旋转主导时，才允许 PnP 改写输出。
    if (frame_scale > kMaxOutputPnPScale) {
        return false;
    }
    if (pnp_inlier_count < kPreferredKeyframePnPInliers) {
        return false;
    }

    const double translation_limit = std::max(0.2, frame_scale * kTrustedPnPTranslationDeltaScale);
    const double translation_delta = norm(pnp_pose.translation - reference_pose.translation);
    if (translation_delta > translation_limit) {
        return false;
    }

    const double rotation_delta = rotationAngleDegrees(pnp_pose.rotation, reference_pose.rotation);
    if (rotation_delta > kTrustedPnPRotationDeltaDegrees) {
        return false;
    }

    if (keyframe_map_age < kMinOutputPnPKeyframeAge) {
        return false;
    }
    if (two_view_track_count >= kMaxOutputPnPTrackedFeatures) {
        return false;
    }
    return two_view_rotation_degrees >= kMinOutputPnPRotationDegrees;
}

double medianParallax(const vector<Point2f>& prev_points, const vector<Point2f>& curr_points) {
    // 计算匹配点在图像上的中位视差，用于判断是否适合三角化。
    // 中位数比平均值更稳健，少量误跟踪点不会显著影响判断。
    if (prev_points.size() != curr_points.size() || prev_points.empty()) {
        return 0.0;
    }

    vector<double> parallaxes;
    parallaxes.reserve(prev_points.size());
    for (size_t i = 0; i < prev_points.size(); ++i) {
        parallaxes.push_back(norm(curr_points[i] - prev_points[i]));
    }

    const size_t middle = parallaxes.size() / 2;
    nth_element(parallaxes.begin(), parallaxes.begin() + middle, parallaxes.end());
    double median = parallaxes[middle];
    if (parallaxes.size() % 2 == 0 && middle > 0) {
        nth_element(parallaxes.begin(), parallaxes.begin() + middle - 1, parallaxes.begin() + middle);
        median = 0.5 * (median + parallaxes[middle - 1]);
    }
    return median;
}

bool shouldTriangulateTracks(const vector<Point2f>& prev_points, const vector<Point2f>& curr_points, double baseline) {
    // 三角化要求同时满足：匹配点数量足够、相机真实基线足够、图像视差足够。
    // 基线或视差太小时，三角化深度会非常不稳定，生成的 3D 点不适合 PnP。
    if (prev_points.size() < static_cast<size_t>(kMinTriangulatedLandmarks) || curr_points.size() != prev_points.size()) {
        return false;
    }
    if (baseline <= kMinTriangulationBaseline) {
        return false;
    }
    return medianParallax(prev_points, curr_points) >= kMinTriangulationParallaxPixels;
}

bool triangulateWorldLandmarks(
    const vector<Point2f>& prev_points,
    const vector<Point2f>& curr_points,
    const Mat& camera_matrix,
    const PoseRecord& prev_pose,
    const Mat& relative_rotation,
    const Mat& relative_translation,
    double relative_scale,
    const PoseRecord* curr_pose_override,
    vector<Point3f>& world_points,
    vector<Point2f>* filtered_prev_points = nullptr,
    vector<Point2f>* filtered_curr_points = nullptr,
    int* finite_point_count = nullptr,
    int* positive_depth_count = nullptr) {
    // 这个函数把当前 2D-2D 主干中的匹配点三角化成短寿命 3D 地图点。
    // 这些 3D 点不会形成长期全局地图，只是为了给后续若干帧提供 3D-2D PnP 输入。
    if (prev_points.size() != curr_points.size() || prev_points.empty()) {
        world_points.clear();
        if (filtered_prev_points != nullptr) {
            filtered_prev_points->clear();
        }
        if (filtered_curr_points != nullptr) {
            filtered_curr_points->clear();
        }
        return false;
    }

    const vector<Point2f> input_prev_points = prev_points;
    const vector<Point2f> input_curr_points = curr_points;

    world_points.clear();
    if (finite_point_count != nullptr) {
        *finite_point_count = 0;
    }
    if (positive_depth_count != nullptr) {
        *positive_depth_count = 0;
    }
    if (filtered_prev_points != nullptr) {
        filtered_prev_points->clear();
        filtered_prev_points->reserve(input_prev_points.size());
    }
    if (filtered_curr_points != nullptr) {
        filtered_curr_points->clear();
        filtered_curr_points->reserve(input_curr_points.size());
    }

    const PoseRecord derived_curr_pose = poseFromRelativeMotion(prev_pose, relative_rotation, relative_translation, relative_scale);
    const PoseRecord& curr_pose = curr_pose_override != nullptr ? *curr_pose_override : derived_curr_pose;

    // triangulation 得到的是 prev_pose 相机坐标系下的局部 3D 点。
    // 后面会用 prev_pose 把这些点变换到世界坐标系，方便跨帧 PnP 复用。
    vector<Point3f> local_points;
    Mat scaled_translation = relative_translation.clone();
    scaled_translation *= relative_scale;
    triangulation(
        input_prev_points,
        input_curr_points,
        camera_matrix,
        relative_rotation,
        scaled_translation,
        local_points);
    const int triangulated_count = min(
        static_cast<int>(min(input_prev_points.size(), input_curr_points.size())),
        static_cast<int>(local_points.size()));
    world_points.reserve(triangulated_count);
    for (int i = 0; i < triangulated_count; ++i) {
        const Point3f& point_local = local_points[i];
        // 过滤 NaN/Inf，避免无效三角化点进入后续 PnP。
        if (!std::isfinite(point_local.x) || !std::isfinite(point_local.y) || !std::isfinite(point_local.z)) {
            continue;
        }
        if (finite_point_count != nullptr) {
            ++(*finite_point_count);
        }

        if (point_local.z <= 1e-6f) {
            // 在参考相机后方的点不满足正常成像几何，直接丢弃。
            continue;
        }
        if (positive_depth_count != nullptr) {
            ++(*positive_depth_count);
        }
        if (point_local.z < kMinLandmarkDepth || point_local.z > kMaxLandmarkDepth) {
            // 过近点容易受噪声影响，过远点深度不稳定；只保留合理深度范围内的地图点。
            continue;
        }

        const Matx33d prev_rotation_cw = prev_pose.rotation;
        const Vec3d prev_translation_world = prev_pose.translation;
        const Vec3d point_world_vec =
            prev_rotation_cw * Vec3d(point_local.x, point_local.y, point_local.z) + prev_translation_world;

        // 用两帧重投影误差做最后筛选：
        // 三角化点投回上一帧和当前帧后，都应接近原始光流观测。
        const Point2d reproj_prev = projectWorldPoint(
            Point3d(point_world_vec[0], point_world_vec[1], point_world_vec[2]),
            camera_matrix,
            prev_pose);
        const Point2d reproj_curr = projectWorldPoint(
            Point3d(point_world_vec[0], point_world_vec[1], point_world_vec[2]),
            camera_matrix,
            curr_pose);
        if (!std::isfinite(reproj_prev.x) || !std::isfinite(reproj_prev.y) ||
            !std::isfinite(reproj_curr.x) || !std::isfinite(reproj_curr.y)) {
            continue;
        }
        if (norm(reproj_prev - Point2d(input_prev_points[i])) > kTriangulationReprojectionError ||
            norm(reproj_curr - Point2d(input_curr_points[i])) > kTriangulationReprojectionError) {
            // 重投影误差过大通常意味着光流匹配错误或三角化深度不可靠。
            continue;
        }

        world_points.emplace_back(
            static_cast<float>(point_world_vec[0]),
            static_cast<float>(point_world_vec[1]),
            static_cast<float>(point_world_vec[2]));
        if (filtered_prev_points != nullptr) {
            filtered_prev_points->push_back(input_prev_points[i]);
        }
        if (filtered_curr_points != nullptr) {
            filtered_curr_points->push_back(input_curr_points[i]);
        }
    }

    return !world_points.empty();
}

void compactTracksByIndices(
    vector<Point2f>& prev_points,
    vector<Point2f>& curr_points,
    vector<Point3f>& world_points,
    const vector<int>& inlier_indices) {
    // PnP RANSAC 返回内点下标后，需要同步压缩 2D 轨迹和 3D 地图点。
    // 这样 keyframe_features、当前观测和 world_points 仍保持同一索引对应同一个空间点。
    vector<Point2f> filtered_prev;
    vector<Point2f> filtered_curr;
    vector<Point3f> filtered_world;
    filtered_prev.reserve(inlier_indices.size());
    filtered_curr.reserve(inlier_indices.size());
    filtered_world.reserve(inlier_indices.size());

    for (int index : inlier_indices) {
        if (index < 0 ||
            index >= static_cast<int>(prev_points.size()) ||
            index >= static_cast<int>(curr_points.size()) ||
            index >= static_cast<int>(world_points.size())) {
            continue;
        }
        filtered_prev.push_back(prev_points[index]);
        filtered_curr.push_back(curr_points[index]);
        filtered_world.push_back(world_points[index]);
    }

    prev_points.swap(filtered_prev);
    curr_points.swap(filtered_curr);
    world_points.swap(filtered_world);
}

bool solvePoseWithPnP(
    const vector<Point3f>& world_points,
    const vector<Point2f>& image_points,
    const Mat& camera_matrix,
    const PoseRecord& initial_guess,
    PoseRecord& pose,
    vector<int>& inlier_indices) {
    // PnP 两步走：
    // 1. 先用 solvePnPRansac 从 3D-2D 对应中剔除外点。
    // 2. 再只用内点做一次 iterative refine，得到更稳定的相机位姿。
    // initial_guess 来自当前 2D-2D 位姿，用于把 PnP 解限制在“接近主干解”的局部区域。
    inlier_indices.clear();
    if (world_points.size() < 4 || image_points.size() < 4 || world_points.size() != image_points.size()) {
        return false;
    }

    Mat rvec;
    Mat tvec;
    poseToPnPGuess(initial_guess, rvec, tvec);
    vector<int> raw_inliers;
    const bool solved = solvePnPRansac(
        world_points,
        image_points,
        camera_matrix,
        noArray(),
        rvec,
        tvec,
        true,
        100,
        kPnPReprojectionError,
        0.99,
        raw_inliers,
        SOLVEPNP_EPNP);
    if (!solved || static_cast<int>(raw_inliers.size()) < kMinPnPInliers) {
        return false;
    }

    vector<Point3f> inlier_world_points;
    vector<Point2f> inlier_image_points;
    inlier_world_points.reserve(raw_inliers.size());
    inlier_image_points.reserve(raw_inliers.size());
    for (int index : raw_inliers) {
        // OpenCV 返回的是原输入数组的下标，先做边界检查再取内点。
        if (index < 0 || index >= static_cast<int>(world_points.size()) || index >= static_cast<int>(image_points.size())) {
            continue;
        }
        inlier_world_points.push_back(world_points[index]);
        inlier_image_points.push_back(image_points[index]);
    }

    if (static_cast<int>(inlier_world_points.size()) < kMinPnPInliers) {
        return false;
    }

    if (!solvePnP(
            inlier_world_points,
            inlier_image_points,
            camera_matrix,
            noArray(),
            rvec,
            tvec,
            true,
            SOLVEPNP_ITERATIVE)) {
        return false;
    }

    // refine 后的 rvec/tvec 转回世界坐标系下的 PoseRecord。
    pose = poseFromPnP(rvec, tvec);
    inlier_indices.swap(raw_inliers);
    return true;
}

Point2f projectTracePoint(
    double x,
    double z,
    double min_x,
    double min_z,
    double scale,
    double offset_x,
    double offset_y) {
    // 轨迹图绘制在 X-Z 平面上，Y 轴方向用画布坐标翻转后显示。
    // 输入的 x、z 是真实坐标；min_x/min_z 用于把轨迹平移到画布左下角附近；
    // scale 把米级坐标转换为像素；offset_x/offset_y 让轨迹在画布中居中。
    return Point2f(
        static_cast<float>(offset_x + (x - min_x) * scale),
        static_cast<float>(kTraceCanvasSize - offset_y - (z - min_z) * scale));
}

bool shouldReportProgress(int processed_frames, int total_frames) {
    // 前几帧、固定间隔以及最后一帧打印进度，避免长序列刷屏。
    return processed_frames <= 5 ||
           processed_frames % kProgressReportInterval == 0 ||
           processed_frames == total_frames;
}

Mat renderTraceCanvas(
    const vector<Vec3d>& gt_positions,
    const vector<PoseRecord>& estimated_poses,
    const vector<int>& frame_ids,
    int processed_frames,
    int total_frames) {
    // 根据当前已经累计的真值和估计位姿动态缩放画布，保证完整轨迹可见。
    // gt_positions 使用原始 frame_id 索引；estimated_positions 只保存成功推进或记录的估计点，
    // 因此需要 frame_ids 记录每个估计点对应的真实帧号。
    Mat trace(kTraceCanvasSize, kTraceCanvasSize, CV_8UC3, Scalar(255, 255, 255));
    putText(trace, "Black--Ground Truth", Point2f(10, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1, 8);
    putText(trace, "Green--VO", Point2f(10, 50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);

    if (estimated_poses.empty() || frame_ids.empty()) {
        return trace;
    }

    double min_x = numeric_limits<double>::max();
    double max_x = numeric_limits<double>::lowest();
    double min_z = numeric_limits<double>::max();
    double max_z = numeric_limits<double>::lowest();

    // 同时统计真值轨迹和 VO 轨迹的范围，用同一个比例尺绘制两条曲线。
    // 如果两条轨迹尺度差异较大，仍以联合边界为准，方便直接观察漂移和方向偏差。
    for (size_t i = 0; i < estimated_poses.size(); ++i) {
        const int frame_id = frame_ids[i];
        const Vec3d& gt = gt_positions[frame_id];
        const Vec3d& est = estimated_poses[i].translation;
        min_x = min(min_x, min(gt[0], est[0]));
        max_x = max(max_x, max(gt[0], est[0]));
        min_z = min(min_z, min(gt[2], est[2]));
        max_z = max(max_z, max(gt[2], est[2]));
    }

    const double range_x = max(1.0, max_x - min_x);
    const double range_z = max(1.0, max_z - min_z);
    // 至少保留 1.0 的范围，避免所有点几乎重合时出现除零或极端放大。
    // drawable 是扣除边距后的实际绘制区域，scale 取 X/Z 两个方向中更小的比例以完整容纳轨迹。
    const double drawable = static_cast<double>(kTraceCanvasSize) - 2.0 * kTracePadding;
    const double scale = min(drawable / range_x, drawable / range_z);
    const double used_width = range_x * scale;
    const double used_height = range_z * scale;
    const double offset_x = (static_cast<double>(kTraceCanvasSize) - used_width) * 0.5;
    const double offset_y = (static_cast<double>(kTraceCanvasSize) - used_height) * 0.5;

    // 黑色点是真值位置，绿色点是视觉里程计估计位置。
    // 这里逐点绘制而不是连线，能更直观看出每一帧的采样密度和跳变情况。
    for (size_t i = 0; i < estimated_poses.size(); ++i) {
        const int frame_id = frame_ids[i];
        const Vec3d& gt = gt_positions[frame_id];
        const Vec3d& est = estimated_poses[i].translation;

        circle(trace, projectTracePoint(gt[0], gt[2], min_x, min_z, scale, offset_x, offset_y), 1, Scalar(0, 0, 0), 1);
        circle(trace, projectTracePoint(est[0], est[2], min_x, min_z, scale, offset_x, offset_y), 1, Scalar(0, 255, 0), 1);
    }

    char info[256];
    std::snprintf(
        info,
        sizeof(info),
        "Frames: %d/%d  X:[%.1f, %.1f]  Z:[%.1f, %.1f]",
        processed_frames,
        total_frames,
        min_x,
        max_x,
        min_z,
        max_z);
    putText(trace, info, Point2f(10, static_cast<float>(kTraceCanvasSize - 15)), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(80, 80, 80), 1, 8);

    return trace;
}

void printUsage(const char* program_name) {
    // 打印命令行用法。支持整段运行，也支持指定起止帧或指定帧数运行。
    cout << "Usage:\n"
         << "  " << program_name << " [frame_count]\n"
         << "  " << program_name << " --start-frame N --end-frame M [--output-root REL_OR_ABS_PATH]\n"
         << "  " << program_name << " --start-frame N --frame-count K [--output-root REL_OR_ABS_PATH]\n\n"
         << "Options:\n"
         << "  --start-frame N   First frame index in the selected segment. Default: 0\n"
         << "  --end-frame M     Last frame index in the selected segment (inclusive).\n"
         << "  --frame-count K   Process K frames starting from --start-frame.\n"
         << "  --output-root P   Output root directory. Default: output/full_run/\n"
         << "  --help            Show this help message.\n";
}

bool parseNonNegativeInt(const string& text, const string& option_name, int& value, string& error_message) {
    // 使用 strtol 严格解析非负整数，确保整段字符串都被消费。
    // 这样 "12abc"、负数和超出 int 范围的值都会被明确拒绝。
    char* end_ptr = nullptr;
    errno = 0;
    const long parsed = std::strtol(text.c_str(), &end_ptr, 10);
    if (errno != 0 || end_ptr == text.c_str() || *end_ptr != '\0') {
        error_message = "Invalid integer for " + option_name + ": " + text;
        return false;
    }
    if (parsed < 0 || parsed > INT_MAX) {
        error_message = "Out-of-range integer for " + option_name + ": " + text;
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool parseRunOptions(int argc, char** argv, RunOptions& options, bool& show_help, string& error_message) {
    // 手写轻量参数解析，避免为几个简单选项引入额外依赖。
    // 位置参数 frame_count 和 --frame-count 二者只允许出现一个。
    for (int i = 1; i < argc; ++i) {
        const string argument(argv[i]);
        if (argument == "--help" || argument == "-h") {
            show_help = true;
            return true;
        }
        if (argument == "--start-frame" || argument == "--end-frame" || argument == "--frame-count" || argument == "--output-root") {
            // 这些选项都要求后面跟一个值。
            if (i + 1 >= argc) {
                error_message = "Missing value for option: " + argument;
                return false;
            }
            const string value(argv[++i]);
            if (argument == "--start-frame") {
                if (!parseNonNegativeInt(value, argument, options.start_frame, error_message)) {
                    return false;
                }
            } else if (argument == "--end-frame") {
                if (!parseNonNegativeInt(value, argument, options.end_frame, error_message)) {
                    return false;
                }
            } else if (argument == "--frame-count") {
                if (!parseNonNegativeInt(value, argument, options.frame_count, error_message)) {
                    return false;
                }
                if (options.frame_count == 0) {
                    error_message = "--frame-count must be greater than 0.";
                    return false;
                }
            } else {
                options.output_root_override = value;
            }
            continue;
        }
        if (!argument.empty() && argument[0] == '-') {
            // 以 - 开头但不在白名单中的参数视为未知选项。
            error_message = "Unknown option: " + argument;
            return false;
        }
        if (options.frame_count >= 0) {
            // 防止用户同时写多个位置参数导致范围语义不清。
            error_message = "Only one positional frame_count argument is supported.";
            return false;
        }
        if (!parseNonNegativeInt(argument, "frame_count", options.frame_count, error_message)) {
            return false;
        }
        if (options.frame_count == 0) {
            error_message = "frame_count must be greater than 0.";
            return false;
        }
    }
    return true;
}

Vec4d rotationToTumQuaternion(const Matx33d& rotation) {
    // TUM 轨迹使用 qx qy qz qw 顺序；这里从累计旋转矩阵转成单位四元数。
    // 根据矩阵迹选择数值更稳定的分支，避免在某些旋转角附近除以很小的数。
    const double trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;

    if (trace > 0.0) {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * scale;
        qx = (rotation(2, 1) - rotation(1, 2)) / scale;
        qy = (rotation(0, 2) - rotation(2, 0)) / scale;
        qz = (rotation(1, 0) - rotation(0, 1)) / scale;
    } else if (rotation(0, 0) > rotation(1, 1) && rotation(0, 0) > rotation(2, 2)) {
        const double scale = std::sqrt(1.0 + rotation(0, 0) - rotation(1, 1) - rotation(2, 2)) * 2.0;
        qw = (rotation(2, 1) - rotation(1, 2)) / scale;
        qx = 0.25 * scale;
        qy = (rotation(0, 1) + rotation(1, 0)) / scale;
        qz = (rotation(0, 2) + rotation(2, 0)) / scale;
    } else if (rotation(1, 1) > rotation(2, 2)) {
        const double scale = std::sqrt(1.0 + rotation(1, 1) - rotation(0, 0) - rotation(2, 2)) * 2.0;
        qw = (rotation(0, 2) - rotation(2, 0)) / scale;
        qx = (rotation(0, 1) + rotation(1, 0)) / scale;
        qy = 0.25 * scale;
        qz = (rotation(1, 2) + rotation(2, 1)) / scale;
    } else {
        const double scale = std::sqrt(1.0 + rotation(2, 2) - rotation(0, 0) - rotation(1, 1)) * 2.0;
        qw = (rotation(1, 0) - rotation(0, 1)) / scale;
        qx = (rotation(0, 2) + rotation(2, 0)) / scale;
        qy = (rotation(1, 2) + rotation(2, 1)) / scale;
        qz = 0.25 * scale;
    }

    const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (norm <= 0.0) {
        return Vec4d(0.0, 0.0, 0.0, 1.0);
    }
    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;
    if (qw < 0.0) {
        // 四元数 q 和 -q 表示同一个旋转，这里固定 qw >= 0，便于输出更连续。
        qx = -qx;
        qy = -qy;
        qz = -qz;
        qw = -qw;
    }
    return Vec4d(qx, qy, qz, qw);
}

}  // namespace

bool loadCameraIntrinsics(const string& camera_info_path, CameraIntrinsics& intrinsics) {
    // 从配置文件读取针孔相机内参，只关心 fx、fy、cx、cy 四个字段。
    // fx/fy 是水平和垂直方向焦距，cx/cy 是主点坐标；这些值用于构造相机矩阵 K。
    // 若 fx 或 fy 没有读到有效正数，后续本质矩阵估计会失去尺度基准，因此返回 false。
    ifstream input(camera_info_path);
    if (!input.is_open()) {
        cerr << "Unable to open camera info: " << camera_info_path << endl;
        return false;
    }

    string line;
    while (getline(input, line)) {
        // rfind(prefix, 0) 用于判断一行是否以指定字段名开头。
        // 冒号后的字符串交给 stod 转成 double，允许配置文件中冒号后带空格。
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
    // TUM 格式每行包含 timestamp tx ty tz qx qy qz qw，本程序只用平移作为真值位置。
    // 这里没有使用四元数，因为单目 VO 主循环只需要相邻帧真值位置距离来恢复绝对尺度。
    // 返回 vector 的下标与图像 frame_id 对齐：positions[10] 对应第 10 帧的真值位置。
    ifstream input(pose_path);
    vector<Vec3d> positions;
    if (!input.is_open()) {
        cerr << "Unable to open pose file: " << pose_path << endl;
        return positions;
    }

    string line;
    while (getline(input, line)) {
        // 空行和以 # 开头的注释行跳过，便于兼容常见 TUM 轨迹文件。
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

vector<double> loadGroundTruthTimestamps(const string& pose_path) {
    // evo 的 TUM 格式需要时间戳；这里从同一个真值轨迹文件提取，保证 frame_id 与位置索引一致。
    ifstream input(pose_path);
    vector<double> timestamps;
    if (!input.is_open()) {
        cerr << "Unable to open pose file: " << pose_path << endl;
        return timestamps;
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
            timestamps.push_back(timestamp);
        }
    }

    return timestamps;
}

Mat buildCameraMatrix(const CameraIntrinsics& intrinsics) {
    // 将内参结构体转为 OpenCV 标准 3x3 相机矩阵 K。
    // K = [fx 0 cx; 0 fy cy; 0 0 1]，用于像素坐标和归一化相机坐标之间转换。
    return (Mat_<double>(3, 3) <<
        intrinsics.fx, 0.0, intrinsics.cx,
        0.0, intrinsics.fy, intrinsics.cy,
        0.0, 0.0, 1.0);
}

void scaleCameraIntrinsics(CameraIntrinsics& intrinsics, double scale) {
    // 图像缩放后，焦距和主点坐标必须按相同比例缩放。
    // 例如图像宽高缩小到 0.5，像素坐标也缩小到 0.5，因此 fx、fy、cx、cy 都要乘 0.5。
    // 如果只缩放图像不缩放内参，findEssentialMat/recoverPose 会使用错误的相机模型。
    intrinsics.fx *= scale;
    intrinsics.fy *= scale;
    intrinsics.cx *= scale;
    intrinsics.cy *= scale;
}

string formatImagePath(const string& image_pattern, int frame_id) {
    // 将 %06d.png 这类格式化模板展开为具体帧图像路径。
    // KITTI 序列通常以六位数字命名，例如 frame_id=7 会变成 000007.png。
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer), image_pattern.c_str(), frame_id);
    return string(buffer);
}

Mat readImageOrEmpty(const string& image_pattern, int frame_id) {
    // 单目 VO 只需要灰度图，读取失败时返回空 Mat 供调用方处理。
    // 灰度图足够用于 FAST 角点和 LK 光流，也能减少内存和计算开销。
    return imread(formatImagePath(image_pattern, frame_id), IMREAD_GRAYSCALE);
}

void featureDetection(const Mat& image, vector<Point2f>& points, int max_corners) {
    // 使用 FAST 提取角点，并保留响应值最高的一批点。
    // FAST 阈值 20 表示中心点与圆周像素的亮度差需要足够明显；true 表示启用非极大值抑制，
    // 可以减少同一局部区域内过于密集的重复角点。
    static Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(20, true);
    vector<KeyPoint> keypoints;
    detector->detect(image, keypoints);
    if (static_cast<int>(keypoints.size()) > max_corners) {
        // nth_element 只保证前 max_corners 个点响应值较大，复杂度低于完整排序。
        // 对 VO 来说只需要稳定且足够多的点，不需要把所有特征严格按响应值排序。
        nth_element(
            keypoints.begin(),
            keypoints.begin() + max_corners,
            keypoints.end(),
            [](const KeyPoint& lhs, const KeyPoint& rhs) { return lhs.response > rhs.response; });
        keypoints.resize(max_corners);
    }
    KeyPoint::convert(keypoints, points);
    if (!points.empty()) {
        // 亚像素角点优化可以提升光流跟踪和后续几何估计的稳定性。
        // Size(10,10) 是搜索窗口，Size(-1,-1) 表示没有死区；
        // 迭代最多 20 次或移动量小于 0.03 像素时停止。
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
    // LK 金字塔光流先从前一帧跟踪到当前帧，再反向跟踪回来做一致性检查。
    // 正向跟踪得到 prev_points 在 curr_image 中的位置 curr_points；
    // 反向跟踪再把 curr_points 跟回 prev_image，如果回不到原来的位置，说明该点不稳定或被遮挡。
    vector<float> errors;
    vector<Point2f> backward_points;
    vector<uchar> backward_status;
    vector<float> backward_errors;
    static const Size kWindowSize(21, 21);
    static const TermCriteria kTermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
    // 金字塔层数为 3，能够处理一定范围的相机运动；窗口 21x21 用于局部灰度匹配。
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
        // 同时过滤：正向失败、反向失败、越界点以及前后向误差过大的点。
        // 过滤后 prev_points 和 curr_points 仍然是一一对应关系，后续才能用于本质矩阵估计。
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

int featureTrackingWithLandmarks(
    const Mat& prev_image,
    const Mat& curr_image,
    vector<Point2f>& prev_points,
    vector<Point2f>& curr_points,
    vector<Point3f>& landmarks,
    vector<uchar>& status) {
    // 和普通 LK 跟踪不同，这里要同时维护 2D 观测与对应的 3D 地图点。
    // 一旦某个点前后向跟踪失败、越界或不稳定，就连同它绑定的 3D 点一起丢弃，
    // 这样传给 PnP 的 3D-2D 对应始终保持同一索引一一匹配。
    if (prev_points.size() != landmarks.size()) {
        prev_points.clear();
        curr_points.clear();
        landmarks.clear();
        return 0;
    }

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

    vector<Point2f> filtered_prev_points;
    vector<Point2f> filtered_curr_points;
    vector<Point3f> filtered_landmarks;
    filtered_prev_points.reserve(prev_points.size());
    filtered_curr_points.reserve(curr_points.size());
    filtered_landmarks.reserve(landmarks.size());

    const int width = curr_image.cols;
    const int height = curr_image.rows;
    for (size_t i = 0; i < status.size() && i < landmarks.size(); ++i) {
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
        filtered_prev_points.push_back(prev_points[i]);
        filtered_curr_points.push_back(point);
        filtered_landmarks.push_back(landmarks[i]);
    }

    prev_points.swap(filtered_prev_points);
    curr_points.swap(filtered_curr_points);
    landmarks.swap(filtered_landmarks);
    return static_cast<int>(curr_points.size());
}

void compactPointsByMask(vector<Point2f>& prev_points, vector<Point2f>& curr_points, const Mat& mask) {
    // findEssentialMat/recoverPose 的 mask 标记内点，这里同步压缩两帧对应点。
    // mask 中非零值表示该匹配点符合估计出的极几何关系；外点通常来自误匹配、动态物体或遮挡。
    // 保留内点能让下一帧继续跟踪的特征更可靠，降低错误匹配累积。
    vector<Point2f> filtered_prev;
    vector<Point2f> filtered_curr;
    filtered_prev.reserve(prev_points.size());
    filtered_curr.reserve(curr_points.size());

    const int mask_count = max(mask.rows, mask.cols);
    for (int i = 0; i < mask_count && i < static_cast<int>(prev_points.size()) && i < static_cast<int>(curr_points.size()); ++i) {
        // OpenCV 的 mask 可能是 1xN，也可能是 Nx1，这里兼容两种形状。
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
    // 像素坐标归一化到相机坐标平面，去掉焦距和主点影响。
    // 公式为 x=(u-cx)/fx，y=(v-cy)/fy。归一化后点可以直接配合 [R|t] 投影矩阵使用。
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
    // 以第一帧为参考坐标系，第二帧使用 recoverPose 得到的相对位姿。
    // 注意：当前主流程没有调用 triangulation，这个函数保留用于三角化特征点或调试深度。
    // pose_1 是 [I|0]，pose_2 是 [R|t]，两帧归一化坐标通过线性三角化得到 3D 点。
    Mat pose_1 = (Mat_<float>(3, 4) <<
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f);
    Mat pose_2 = (Mat_<float>(3, 4) <<
        static_cast<float>(rotation.at<double>(0, 0)), static_cast<float>(rotation.at<double>(0, 1)), static_cast<float>(rotation.at<double>(0, 2)), static_cast<float>(translation.at<double>(0, 0)),
        static_cast<float>(rotation.at<double>(1, 0)), static_cast<float>(rotation.at<double>(1, 1)), static_cast<float>(rotation.at<double>(1, 2)), static_cast<float>(translation.at<double>(1, 0)),
        static_cast<float>(rotation.at<double>(2, 0)), static_cast<float>(rotation.at<double>(2, 1)), static_cast<float>(rotation.at<double>(2, 2)), static_cast<float>(translation.at<double>(2, 0)));

    Mat normalized_1(2, static_cast<int>(points_1.size()), CV_32F);
    Mat normalized_2(2, static_cast<int>(points_2.size()), CV_32F);

    // triangulatePoints 输入的是归一化坐标，因此投影矩阵中不再乘相机内参 K。
    // 如果使用像素坐标，则投影矩阵需要写成 K[I|0] 和 K[R|t]；这里选择先归一化点。
    for (size_t i = 0; i < points_1.size(); ++i) {
        const Point2f point_1 = pixel2cam(points_1[i], camera_matrix);
        const Point2f point_2 = pixel2cam(points_2[i], camera_matrix);
        normalized_1.at<float>(0, static_cast<int>(i)) = point_1.x;
        normalized_1.at<float>(1, static_cast<int>(i)) = point_1.y;
        normalized_2.at<float>(0, static_cast<int>(i)) = point_2.x;
        normalized_2.at<float>(1, static_cast<int>(i)) = point_2.y;
    }

    Mat points_4d;
    triangulatePoints(pose_1, pose_2, normalized_1, normalized_2, points_4d);

    points.clear();
    for (int i = 0; i < points_4d.cols; ++i) {
        // 齐次坐标除以 w，得到三维欧式坐标。
        // points_4d 每一列是 [X, Y, Z, W]^T，归一化后保存为 Point3f。
        Mat column = points_4d.col(i);
        const float w = column.at<float>(3, 0);
        if (!std::isfinite(w) || std::abs(w) <= 1e-9f) {
            continue;
        }
        column /= w;
        points.emplace_back(
            column.at<float>(0, 0),
            column.at<float>(1, 0),
            column.at<float>(2, 0));
    }
}

double getAbsoluteScale(const vector<Vec3d>& gt_positions, int frame_id) {
    // 单目 VO 只能恢复平移方向，绝对尺度用相邻真值位姿距离补充。
    // recoverPose 给出的 translation 通常是单位长度方向向量，无法知道相机真实移动了多少米。
    // 这里用真值轨迹中 frame_id-1 到 frame_id 的欧氏距离作为尺度，属于带真值辅助的 VO。
    if (frame_id <= 0 || frame_id >= static_cast<int>(gt_positions.size())) {
        return 0.0;
    }

    const Vec3d& prev = gt_positions[frame_id - 1];
    const Vec3d& curr = gt_positions[frame_id];
    const Vec3d delta = curr - prev;
    return sqrt(delta.dot(delta));
}

void appendPose(ofstream& output, double timestamp, const PoseRecord& pose) {
    // 输出 evo 可直接读取的 TUM 格式：timestamp tx ty tz qx qy qz qw。
    const Vec4d quaternion = rotationToTumQuaternion(pose.rotation);
    output << setprecision(18)
           << timestamp << ' '
           << pose.translation[0] << ' '
           << pose.translation[1] << ' '
           << pose.translation[2] << ' '
           << quaternion[0] << ' '
           << quaternion[1] << ' '
           << quaternion[2] << ' '
           << quaternion[3] << '\n';
}

bool isVisualizationEnabled() {
    // 可通过 VO_ENABLE_GUI=0 强制关闭窗口显示，适合无桌面环境运行。
    // 如果没有显式关闭，则分别检查 X11 和 Wayland 是否可连接；只有可用时才创建 OpenCV 窗口。
    const char* explicit_gui = std::getenv("VO_ENABLE_GUI");
    if (explicit_gui != nullptr && std::string(explicit_gui) == "0") {
        return false;
    }

    const char* display = std::getenv("DISPLAY");
    const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
    return isAccessibleX11Display(display) || isAccessibleWaylandDisplay(wayland_display);
}

bool isVerboseLoggingEnabled() {
    // VO_VERBOSE_LOG=1 时打印更详细的失败和重初始化信息。
    // 默认只打印关键进度，避免处理长序列时输出过多。
    const char* verbose = std::getenv("VO_VERBOSE_LOG");
    return verbose != nullptr && std::string(verbose) == "1";
}

bool isPnPEnabled() {
    const char* pnp = std::getenv("VO_ENABLE_PNP");
    if (pnp == nullptr) {
        return true;
    }
    return std::string(pnp) != "0";
}

double getImageScale() {
    // VO_IMAGE_SCALE 允许在 0 到 1 之间缩小输入图像，加快处理速度。
    // 合法范围是 (0, 1]：大于 1 会放大图像增加计算量，非正数没有意义，因此回退到默认比例。
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
    // 初始化路径、相机内参和真值轨迹。
    // 程序的整体流程是：
    // 1. 读取相机内参和真值位置。
    // 2. 用前两帧初始化特征、光流和相对位姿。
    // 3. 从第 2 帧开始逐帧跟踪特征、估计帧间运动、累计全局轨迹。
    // 4. 输出 position.txt 和轨迹对比图 map2.png。
    RunOptions run_options;
    bool show_help = false;
    string option_error_message;
    if (!parseRunOptions(argc, argv, run_options, show_help, option_error_message)) {
        cerr << option_error_message << endl;
        printUsage(argv[0]);
        return -1;
    }
    if (show_help) {
        printUsage(argv[0]);
        return 0;
    }

    const ProjectPaths paths = makeProjectPaths(run_options.output_root_override);
    CameraIntrinsics intrinsics;
    if (!loadCameraIntrinsics(paths.camera_info_path, intrinsics)) {
        return -1;
    }

    const double image_scale = getImageScale();
    // 如果图像被缩放，必须先缩放内参，再构造 camera_matrix。
    // 后续 findEssentialMat 使用 focal_length/principal_point，triangulation 使用完整 K。
    scaleCameraIntrinsics(intrinsics, image_scale);
    const Mat camera_matrix = buildCameraMatrix(intrinsics);
    const vector<Vec3d> gt_positions = loadGroundTruthPositions(paths.gt_pose_path);
    const vector<double> gt_timestamps = loadGroundTruthTimestamps(paths.gt_pose_path);
    const int available_frames = static_cast<int>(min(gt_positions.size(), gt_timestamps.size()));
    if (available_frames < 2) {
        cerr << "Ground truth pose count is insufficient." << endl;
        return -1;
    }

    // 根据命令行参数确定本次处理的帧区间。
    // start_frame/end_frame 使用原始数据集帧号，便于截取任意子序列调试。
    const int segment_start_frame = run_options.start_frame;
    if (segment_start_frame < 0 || segment_start_frame >= available_frames - 1) {
        cerr << "start-frame must leave at least two available frames inside the sequence." << endl;
        return -1;
    }

    int segment_end_frame = available_frames - 1;
    if (run_options.end_frame >= 0) {
        // end_frame 是闭区间终点，超过真值长度时自动截断到最后一帧。
        segment_end_frame = min(run_options.end_frame, available_frames - 1);
    }
    if (run_options.frame_count > 0) {
        // frame_count 与 end_frame 同时给出时，取二者更短的范围。
        const long long capped_end = static_cast<long long>(segment_start_frame) + run_options.frame_count - 1;
        const int frame_count_end = capped_end > available_frames - 1
            ? available_frames - 1
            : static_cast<int>(capped_end);
        segment_end_frame = min(segment_end_frame, frame_count_end);
    }
    if (segment_end_frame <= segment_start_frame) {
        cerr << "Selected range must contain at least two frames." << endl;
        return -1;
    }

    const int segment_frame_count = segment_end_frame - segment_start_frame + 1;

    // 用前两帧完成初始特征跟踪、相对位姿估计和首批地图点三角化。
    // 第 0 帧检测 FAST 特征，第 1 帧用 LK 光流找到对应点；
    // 有了两帧对应点后先估计本质矩阵和相对位姿，再三角化出后续 PnP 所需的 3D 点。
    Mat img_1 = readAndResizeImage(paths.left_image_pattern, segment_start_frame, image_scale);
    Mat img_2 = readAndResizeImage(paths.left_image_pattern, segment_start_frame + 1, image_scale);
    if (img_1.empty() || img_2.empty()) {
        cerr << "Error reading initial images." << endl;
        return -1;
    }

    vector<Point2f> points1;
    vector<Point2f> points2;
    // points1 保存第 0 帧特征点；points2 将保存这些点在第 1 帧中的对应位置。
    featureDetection(img_1, points1, kTargetFeatureCount);
    points2.reserve(points1.size());

    vector<uchar> status;
    featureTracking(img_1, img_2, points1, points2, status);
    if (points1.size() < 8 || points2.size() < 8) {
        // 本质矩阵理论上至少需要 5 个点，实际 RANSAC 和 recoverPose 需要更多点才稳定。
        // 这里以 8 个作为最低保护阈值，低于该数量直接初始化失败。
        cerr << "Not enough tracked features after initialization." << endl;
        return -1;
    }

    const double focal_length = intrinsics.fx;
    const Point2d principal_point(intrinsics.cx, intrinsics.cy);
    // OpenCV 这个 findEssentialMat 重载使用一个焦距和主点，默认假设 fx 与 fy 接近。
    // 当前数据通常满足该假设；若相机 fx/fy 差异很大，应改用完整 camera_matrix 重载。

    Mat essential_matrix;
    Mat rotation;
    Mat translation;
    Mat mask;
    // RANSAC 估计本质矩阵，recoverPose 从本质矩阵恢复相对旋转和平移方向。
    // RANSAC 参数含义：
    // - 0.999：希望估计结果正确的置信度。
    // - 1.0：像素误差阈值，超过该误差的匹配会被认为是外点。
    // mask 会记录哪些匹配点最终被当作内点。
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point, RANSAC, 0.999, 1.0, mask);
    const int initial_inliers =
        recoverPose(essential_matrix, points1, points2, rotation, translation, focal_length, principal_point, mask);
    if (initial_inliers < kMinInitialPoseInliers) {
        // 初始位姿不可靠时不能进入主循环，否则后续累计轨迹会从错误方向开始漂移。
        cerr << "Not enough pose inliers after initialization: "
             << initial_inliers << " inliers from " << points1.size() << " tracked features." << endl;
        return -1;
    }
    compactPointsByMask(points1, points2, mask);

    const bool enable_pnp = isPnPEnabled();
    const double initial_scale = getAbsoluteScale(gt_positions, segment_start_frame + 1);
    // geometry_pose 是内部几何位姿，用于三角化、PnP 初值和一致性检查。
    // output_pose 是最终输出轨迹位姿，保留原 2D-2D 轨迹累计方式，并允许有限 PnP 姿态修正。
    PoseRecord geometry_pose = poseFromInitialRelativeMotion(rotation, translation, initial_scale);
    PoseRecord output_pose = poseFromInitialOutputMotion(rotation, translation, initial_scale);

    vector<Point2f> init_prev_features = points1;
    vector<Point2f> init_curr_features = points2;
    vector<Point2f> init_prev_landmark_features = points1;
    vector<Point2f> init_curr_landmark_features = points2;
    vector<Point3f> tracked_landmarks;
    // 如果启用了 PnP，初始化阶段会先尝试用前两帧内点生成第一批短时地图点。
    // 三角化失败并不会终止程序，只是后续暂时只依赖 2D-2D 主干，等视差合适时再重建地图。
    const bool init_triangulated =
        enable_pnp &&
        shouldTriangulateTracks(init_prev_landmark_features, init_curr_landmark_features, initial_scale) &&
        triangulateWorldLandmarks(
            init_prev_landmark_features,
            init_curr_landmark_features,
            camera_matrix,
            PoseRecord(),
            rotation,
            translation,
            initial_scale,
            &geometry_pose,
            tracked_landmarks,
            &init_prev_landmark_features,
            &init_curr_landmark_features);
    if (!init_triangulated || static_cast<int>(tracked_landmarks.size()) < kMinTriangulatedLandmarks) {
        tracked_landmarks.clear();
        init_prev_landmark_features.clear();
        init_curr_landmark_features.clear();
        if (enable_pnp) {
            cerr << "Initialization triangulation is insufficient; the next frame will rebuild landmarks via 2D-2D." << endl;
        }
    }

    // 窗口显示是可选功能：有本地显示服务时实时显示当前图像和轨迹，否则自动无头运行。
    const bool enable_visualization = isVisualizationEnabled();
    const bool verbose_logging = isVerboseLoggingEnabled();
    if (enable_visualization) {
        namedWindow("CAMERA IMAGE", WINDOW_AUTOSIZE);
        namedWindow("Trajectory Drawing", WINDOW_AUTOSIZE);
    } else {
        cout << "Visualization disabled. GUI opens by default when a usable local display socket is present. "
             << "Set VO_ENABLE_GUI=0 to force headless mode." << endl;
    }
    Mat prev_image = img_2;
    Mat curr_image;
    vector<Point2f> prev_features = init_curr_features;
    vector<Point2f> curr_features;
    curr_features.reserve(prev_features.size());
    // prev_image/prev_features 始终是 2D-2D 主干的上一帧状态。
    // keyframe_landmarks 保存当前短寿命地图的 3D 点；
    // keyframe_image/keyframe_features 保存这些 3D 点在“上一帧图像”中的 2D 观测，
    // 下一帧只需要把这些观测连续跟踪到当前帧，再做 3D-2D PnP。
    Mat keyframe_image = img_2;
    vector<Point2f> keyframe_features = init_curr_landmark_features;
    vector<Point3f> keyframe_landmarks = tracked_landmarks;
    PoseRecord keyframe_pose = geometry_pose;
    int keyframe_frame_id = segment_start_frame + 1;
    int keyframe_map_frame_id = segment_start_frame + 1;
    // keyframe_frame_id 表示 keyframe_image 对应的最近一帧；
    // keyframe_map_frame_id 表示当前 3D 地图是在哪一帧重建出来的，用于限制地图寿命。

    // trajectory 和 trajectory_frame_ids 一一对应，用于输出 TUM 轨迹、绘制估计轨迹并索引真值位置。
    // 初始估计来自第 0 到第 1 帧，所以第一条轨迹记录对应 frame_id=1。
    vector<PoseRecord> trajectory;
    trajectory.reserve(segment_frame_count);
    vector<PoseRecord> geometry_trajectory;
    geometry_trajectory.reserve(segment_frame_count);
    vector<int> trajectory_frame_ids;
    trajectory_frame_ids.reserve(segment_frame_count);
    trajectory.push_back(output_pose);
    geometry_trajectory.push_back(geometry_pose);
    trajectory_frame_ids.push_back(segment_start_frame + 1);

    future<Mat> next_frame_future;
    if (segment_start_frame + 2 <= segment_end_frame) {
        // 异步预读下一帧，减少图像读取对主循环的阻塞。
        // 主线程处理当前帧特征和位姿时，后台线程提前把下一张图像读入内存。
        next_frame_future = async(
            std::launch::async,
            readAndResizeImage,
            paths.left_image_pattern,
            segment_start_frame + 2,
            image_scale);
    }

    if (!run_options.output_root_override.empty()) {
        cout << "Custom output root: " << run_options.output_root_override << endl;
    } else {
        cout << "Using default output root: output" << endl;
    }
    if (segment_start_frame == 0 && segment_end_frame == available_frames - 1) {
        cout << "Using all available frames: " << available_frames << endl;
    } else {
        cout << "Selected frame range: [" << segment_start_frame << ", " << segment_end_frame << "]"
             << " (" << segment_frame_count << " frames)" << endl;
    }
    cout << "Processing " << segment_frame_count << " frames..." << endl;

    for (int frame_id = segment_start_frame + 2; frame_id <= segment_end_frame; ++frame_id) {
        // 当前帧优先使用上一轮已经预读完成的结果。
        // 第 2 帧开始进入循环，因为第 0、1 帧已用于初始化。
        curr_image = next_frame_future.valid()
            ? next_frame_future.get()
            : readAndResizeImage(paths.left_image_pattern, frame_id, image_scale);
        if (frame_id + 1 <= segment_end_frame) {
            // 立刻发起下一帧预读，让磁盘 IO 与当前帧计算尽量重叠。
            next_frame_future = async(std::launch::async, readAndResizeImage, paths.left_image_pattern, frame_id + 1, image_scale);
        }
        if (curr_image.empty()) {
            // 某一帧缺失时不终止程序，只跳过该帧；后续帧仍可继续尝试处理。
            cerr << "Skipping missing frame: " << formatImagePath(paths.left_image_pattern, frame_id) << endl;
            continue;
        }

        vector<uchar> tracking_status;
        curr_features.clear();
        if (!prev_features.empty()) {
            // 2D-2D 主干：上一帧特征直接跟踪到当前帧，用于本质矩阵估计。
            featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
        }
        if (prev_features.size() < 8 || curr_features.size() < 8) {
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": not enough tracked features, re-detecting." << endl;
            }
            featureDetection(prev_image, prev_features, kTargetFeatureCount);
            curr_features.clear();
            featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
            if (prev_features.size() < 8 || curr_features.size() < 8) {
                // 重检测后仍没有足够匹配时，本帧不更新任何位姿。
                // 但会在当前帧重新检测特征，为下一帧恢复跟踪做准备。
                trajectory.push_back(output_pose);
                geometry_trajectory.push_back(geometry_pose);
                trajectory_frame_ids.push_back(frame_id);
                prev_image = curr_image;
                featureDetection(curr_image, prev_features, kTargetFeatureCount);
                const int processed_frames = frame_id - segment_start_frame + 1;
                if (shouldReportProgress(processed_frames, segment_frame_count)) {
                    cout << "Frame " << frame_id << " (" << processed_frames << "/" << segment_frame_count << ")"
                         << " tracked=0 inliers=0 status=reinit-failed" << endl;
                }
                continue;
            }
        }

        // 先用 2D-2D 匹配估计本质矩阵，这是整套流程的主干运动估计。
        // 后面的 PnP 只在严格条件下作为辅助修正。
        essential_matrix = findEssentialMat(
            prev_features,
            curr_features,
            focal_length,
            principal_point,
            RANSAC,
            0.999,
            1.0,
            mask);
        int inlier_count = recoverPose(
            essential_matrix,
            prev_features,
            curr_features,
            rotation,
            translation,
            focal_length,
            principal_point,
            mask);
        if (inlier_count < kMinPoseInliers) {
            // 本质矩阵内点不足时，说明当前两帧几何关系不可信。
            // 这里保留上一帧位姿输出，使轨迹长度和帧号仍保持对齐。
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": 2D-2D pose failed." << endl;
            }
            featureDetection(curr_image, curr_features, kTargetFeatureCount);
            trajectory.push_back(output_pose);
            geometry_trajectory.push_back(geometry_pose);
            trajectory_frame_ids.push_back(frame_id);
            prev_image = curr_image;
            prev_features = curr_features;
            const int processed_frames = frame_id - segment_start_frame + 1;
            if (shouldReportProgress(processed_frames, segment_frame_count)) {
                cout << "Frame " << frame_id << " (" << processed_frames << "/" << segment_frame_count << ")"
                     << " tracked=" << curr_features.size()
                     << " inliers=" << inlier_count
                     << " status=skip-pose" << endl;
            }
            continue;
        }

        compactPointsByMask(prev_features, curr_features, mask);
        const double scale = getAbsoluteScale(gt_positions, frame_id);
        if (scale <= kMinTranslationScale) {
            // 真值位移过小的帧没有可靠尺度，直接跳过位姿更新。
            // 这种帧通常对轨迹贡献很小，但会放大 recoverPose 的方向噪声。
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": scale below threshold." << endl;
            }
            trajectory.push_back(output_pose);
            geometry_trajectory.push_back(geometry_pose);
            trajectory_frame_ids.push_back(frame_id);
            prev_image = curr_image;
            prev_features = curr_features;
            const int processed_frames = frame_id - segment_start_frame + 1;
            if (shouldReportProgress(processed_frames, segment_frame_count)) {
                cout << "Frame " << frame_id << " (" << processed_frames << "/" << segment_frame_count << ")"
                     << " tracked=" << curr_features.size()
                     << " inliers=" << inlier_count
                     << " status=skip-small-baseline" << endl;
            }
            continue;
        }

        PoseRecord base_geometry_pose = poseFromRelativeMotion(geometry_pose, rotation, translation, scale);
        PoseRecord estimated_geometry_pose = base_geometry_pose;
        PoseRecord base_output_pose = accumulateOutputPose(output_pose, rotation, translation, scale);
        // output_pose_for_publish 是“当前帧最终写到 trajectory 里的结果”；
        // next_output_pose 是“下一帧继续积分时采用的输出状态”。
        // 当前主干策略里，PnP 即使被允许生效，也只接管输出旋转，不接管平移累计。
        PoseRecord output_pose_for_publish = base_output_pose;
        PoseRecord next_output_pose = base_output_pose;
        string status_text = "2d2d";
        const double two_view_rotation_degrees = rotationAngleDegrees(toMatx33d(rotation.t()), Matx33d::eye());

        vector<Point2f> pnp_prev_features = keyframe_features;
        vector<Point2f> pnp_curr_features;
        vector<Point3f> pnp_landmarks = keyframe_landmarks;
        vector<uchar> pnp_tracking_status;
        const int keyframe_track_age = frame_id - keyframe_frame_id;
        const int keyframe_map_age = frame_id - keyframe_map_frame_id;
        // 只有 PnP 开启、短时地图仍在寿命范围内、且地图点能跟踪出足够 2D 观测时，才尝试 PnP。
        if (enable_pnp &&
            !keyframe_image.empty() &&
            !keyframe_features.empty() &&
            !keyframe_landmarks.empty() &&
            keyframe_track_age <= kMaxKeyframeTrackAge &&
            keyframe_map_age <= kMaxKeyframeTrackAge &&
            featureTrackingWithLandmarks(
                keyframe_image,
                curr_image,
                pnp_prev_features,
                pnp_curr_features,
                pnp_landmarks,
                pnp_tracking_status) >= kMinKeyframePnPInliers) {
            // 这一段是 3D-2D PnP 主路径：
            // 1. 从当前 keyframe 的短时地图出发，把地图点跟踪到当前帧。
            // 2. 用这些 3D-2D 对应求 PnP 位姿。
            // 3. 如果 PnP 通过一致性与门控检查，再决定是否作用到输出轨迹。
            vector<int> pnp_inlier_indices;
            PoseRecord pnp_pose;
            if (solvePoseWithPnP(pnp_landmarks, pnp_curr_features, camera_matrix, base_geometry_pose, pnp_pose, pnp_inlier_indices) &&
                static_cast<int>(pnp_inlier_indices.size()) >= kMinKeyframePnPInliers &&
                isPnPPoseConsistent(pnp_pose, base_geometry_pose, scale)) {
                Mat corrected_relative_rotation;
                Mat corrected_relative_translation;
                double corrected_relative_scale = 0.0;
                compactTracksByIndices(pnp_prev_features, pnp_curr_features, pnp_landmarks, pnp_inlier_indices);
                if (relativeMotionFromPoses(
                        geometry_pose,
                        pnp_pose,
                        corrected_relative_rotation,
                        corrected_relative_translation,
                        corrected_relative_scale)) {
                    // 通过 relativeMotionFromPoses 可把 PnP 绝对位姿转换为和主干相同的相对运动表达。
                    // 当前版本只利用这个检查结果，不直接用它替换 geometry_pose。
                    const double pnp_rotation_delta_deg =
                        rotationAngleDegrees(pnp_pose.rotation, base_geometry_pose.rotation);
                    const double pnp_translation_delta =
                        norm(pnp_pose.translation - base_geometry_pose.translation);
                    if (shouldApplyPnPToOutput(
                            pnp_pose,
                            base_geometry_pose,
                            scale,
                            static_cast<int>(pnp_inlier_indices.size()),
                            keyframe_map_age,
                            static_cast<int>(curr_features.size()),
                            two_view_rotation_degrees)) {
                        // 当前真正“起作用”的位置在这里：
                        // PnP 不改 geometry_pose，也不改平移累计；
                        // 它只把当前输出姿态和后续输出状态的旋转，替换为 PnP 的旋转。
                        output_pose_for_publish = base_output_pose;
                        output_pose_for_publish.rotation = pnp_pose.rotation;
                        next_output_pose = output_pose_for_publish;
                        next_output_pose.translation = base_output_pose.translation;
                        status_text = "pnp";
                    } else {
                        status_text = "pnp-candidate";
                    }
                    if (verbose_logging) {
                        cout << "Frame " << frame_id
                             << ": accepted PnP correction"
                             << " map_age=" << keyframe_map_age
                             << " track_age=" << keyframe_track_age
                             << " landmarks=" << pnp_landmarks.size()
                             << " inliers=" << pnp_inlier_indices.size()
                             << " rot_delta_deg=" << pnp_rotation_delta_deg
                             << " trans_delta_m=" << pnp_translation_delta
                             << " output_applied=" << (status_text == "pnp" ? 1 : 0)
                             << endl;
                    }
                    keyframe_image = curr_image;
                    keyframe_features = pnp_curr_features;
                    keyframe_landmarks = pnp_landmarks;
                    keyframe_pose = base_geometry_pose;
                    keyframe_frame_id = frame_id;
                    // PnP 成功后，把 keyframe_image 推进到当前帧，并只保留 PnP 内点地图。
                    // 这样下一帧继续从当前图像跟踪，减少长距离光流造成的丢点。
                } else if (verbose_logging) {
                    cout << "Frame " << frame_id << ": rejected PnP correction due to degenerate relative motion." << endl;
                }
            } else {
                if (!pnp_curr_features.empty() && !pnp_landmarks.empty()) {
                    // 即使本次 PnP 解没有通过一致性检查，只要地图点仍能跟踪，
                    // 就把 2D 观测推进到当前帧，延长这批短时地图的可用时间。
                    keyframe_image = curr_image;
                    keyframe_features = pnp_curr_features;
                    keyframe_landmarks = pnp_landmarks;
                    keyframe_frame_id = frame_id;
                }
                if (verbose_logging) {
                    cout << "Frame " << frame_id << ": rejected PnP correction, using 2D-2D pose." << endl;
                }
            }
        }

        vector<Point2f> triangulation_prev_features = prev_features;
        vector<Point2f> triangulation_curr_features = curr_features;
        vector<Point3f> rebuilt_landmarks;
        const int rebuild_input_tracks = static_cast<int>(min(
            triangulation_prev_features.size(),
            triangulation_curr_features.size()));
        const bool can_triangulate = shouldTriangulateTracks(triangulation_prev_features, triangulation_curr_features, scale);
        const bool should_refresh_keyframe =
            enable_pnp &&
            (keyframe_landmarks.empty() ||
            static_cast<int>(keyframe_landmarks.size()) < kMinKeyframePnPInliers ||
            keyframe_map_age >= kMaxKeyframeTrackAge);
        // 当地图为空、点数不足或寿命太长时，尝试用当前 2D-2D 内点重新三角化一批地图点。
        if (should_refresh_keyframe &&
            can_triangulate &&
            // 只有 2D-2D 主干在当前帧提供了足够基线和视差，才重建一份新的短时 3D 地图。
            // 这份地图随后成为 PnP 的 keyframe 地图来源。
            triangulateWorldLandmarks(
                triangulation_prev_features,
                triangulation_curr_features,
                camera_matrix,
                geometry_pose,
                rotation,
                translation,
                scale,
                &base_geometry_pose,
                rebuilt_landmarks,
                &triangulation_prev_features,
                &triangulation_curr_features) &&
            static_cast<int>(rebuilt_landmarks.size()) >= kMinKeyframePnPInliers) {
            keyframe_image = curr_image;
            keyframe_features = triangulation_curr_features;
            keyframe_landmarks = rebuilt_landmarks;
            keyframe_pose = estimated_geometry_pose;
            keyframe_frame_id = frame_id;
            keyframe_map_frame_id = frame_id;
            // 重建成功后，当前帧既是新的观测 keyframe，也是这批地图的出生帧。
            if (status_text == "2d2d") {
                status_text = "rebuild-2d2d";
            }
        } else if (should_refresh_keyframe) {
            if (keyframe_landmarks.empty()) {
                // 如果地图已经空了但当前又不能三角化，至少更新 keyframe_image，
                // 等下一帧出现足够视差时再尝试重建。
                keyframe_image = curr_image;
                keyframe_pose = estimated_geometry_pose;
                keyframe_frame_id = frame_id;
            }
            if (verbose_logging && !can_triangulate) {
                cout << "Frame " << frame_id << ": skipped keyframe rebuild due to low baseline/parallax." << endl;
            } else if (verbose_logging && rebuild_input_tracks > 0) {
                cout << "Frame " << frame_id << ": keyframe rebuild produced too few landmarks." << endl;
            }
        }

        geometry_pose = estimated_geometry_pose;
        output_pose = next_output_pose;
        // geometry_pose 始终按 2D-2D 主干推进；
        // output_pose 则可能带有通过门控的 PnP 旋转修正。

        trajectory.push_back(output_pose_for_publish);
        geometry_trajectory.push_back(geometry_pose);
        trajectory_frame_ids.push_back(frame_id);

        if (curr_features.size() < kMinTrackedFeatures) {
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": tracked features down to " << curr_features.size() << ", re-detecting." << endl;
            }
            featureDetection(curr_image, curr_features, kTargetFeatureCount);
        }

        const int processed_frames = frame_id - segment_start_frame + 1;
        if (shouldReportProgress(processed_frames, segment_frame_count)) {
            cout << "Frame " << frame_id << " (" << processed_frames << "/" << segment_frame_count << ")"
                 << " tracked=" << curr_features.size()
                 << " inliers=" << inlier_count
                 << " status=" << status_text
                 << " pose=(" << output_pose_for_publish.translation[0] << ", "
                 << output_pose_for_publish.translation[1] << ", "
                 << output_pose_for_publish.translation[2] << ")" << endl;
        }

        prev_image = curr_image;
        prev_features = curr_features;

        if (enable_visualization) {
            // 实时显示当前灰度图和 X-Z 平面轨迹。waitKey(1) 同时负责刷新 OpenCV 窗口事件。
            Mat trace = renderTraceCanvas(gt_positions, trajectory, trajectory_frame_ids, processed_frames, segment_frame_count);
            imshow("CAMERA IMAGE", curr_image);
            imshow("Trajectory Drawing", trace);
            waitKey(1);
        }
    }

    // 处理结束后保存估计轨迹文本，并生成最终轨迹对比图。
    // position.txt 使用 TUM 格式，可直接用 evo_ape/evo_rpe 评估；map2.png 中黑色为真值，绿色为 VO 估计。
    if (!ensureParentDirectory(paths.output_pose_path) ||
        !ensureParentDirectory(paths.output_geometry_pose_path) ||
        !ensureParentDirectory(paths.output_map_path)) {
        return -1;
    }
    ofstream output(paths.output_pose_path);
    if (!output.is_open()) {
        cerr << "Unable to open output file: " << paths.output_pose_path << endl;
        return -1;
    }
    ofstream geometry_output(paths.output_geometry_pose_path);
    if (!geometry_output.is_open()) {
        cerr << "Unable to open output file: " << paths.output_geometry_pose_path << endl;
        return -1;
    }
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const int frame_id = trajectory_frame_ids[i];
        if (frame_id >= 0 && frame_id < static_cast<int>(gt_timestamps.size())) {
            appendPose(output, gt_timestamps[frame_id], trajectory[i]);
            appendPose(geometry_output, gt_timestamps[frame_id], geometry_trajectory[i]);
        }
    }
    output.close();
    geometry_output.close();
    // 最后一张轨迹图即使在无 GUI 模式下也会保存，方便离线查看。
    Mat trace = renderTraceCanvas(gt_positions, trajectory, trajectory_frame_ids, segment_frame_count, segment_frame_count);
    imwrite(paths.output_map_path, trace);
    cout << "Finished. Saved TUM trajectory to " << paths.output_pose_path
         << ", geometry trajectory to " << paths.output_geometry_pose_path
         << " and map to " << paths.output_map_path << endl;
    return 0;
}
