#include "My_VO.h"

#include <cmath>
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
constexpr int kMinInitialPoseInliers = 8;
constexpr int kMinPoseInliers = 20;

namespace fs = std::filesystem;

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

ProjectPaths makeProjectPaths() {
    // 根据工程根目录统一生成输入输出文件路径，避免依赖固定启动目录。
    // 输入：
    // - Kitti/camera-info.txt：相机内参配置。
    // - Kitti/left/%06d.png：左目灰度图序列，frame_id 会格式化为 000000.png。
    // - Kitti/gt-tum07.txt：TUM 格式真值轨迹。
    // 输出：
    // - output/position/position.txt：本程序估计出的 TUM 格式轨迹。
    // - output/traj/map2.png：本程序绘制的轨迹对比图。
    const fs::path project_root = findProjectRoot();
    const fs::path kitti_dir = project_root / "Kitti";
    const fs::path output_dir = project_root / "output";
    return {
        (kitti_dir / "camera-info.txt").string(),
        (kitti_dir / "left" / "%06d.png").string(),
        (kitti_dir / "gt-tum07.txt").string(),
        (output_dir / "position" / "position.txt").string(),
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

bool shouldReportProgress(int frame_id, int max_frames) {
    // 前几帧、固定间隔以及最后一帧打印进度，避免长序列刷屏。
    // frame_id 从 0 开始，主循环从第 2 帧开始，因此 frame_id + 1 表示已经处理到的帧数量。
    return frame_id < 5 || frame_id % kProgressReportInterval == 0 || frame_id + 1 == max_frames;
}

Mat renderTraceCanvas(
    const vector<Vec3d>& gt_positions,
    const vector<PoseRecord>& estimated_poses,
    const vector<int>& frame_ids,
    int current_frame,
    int max_frames) {
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
        current_frame + 1,
        max_frames,
        min_x,
        max_x,
        min_z,
        max_z);
    putText(trace, info, Point2f(10, static_cast<float>(kTraceCanvasSize - 15)), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(80, 80, 80), 1, 8);

    return trace;
}

Vec4d rotationToTumQuaternion(const Matx33d& rotation) {
    // TUM 轨迹使用 qx qy qz qw 顺序；这里从累计旋转矩阵转成单位四元数。
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

void compactPointsByMask(vector<Point2f>& prev_points, vector<Point2f>& curr_points, const Mat& mask) {
    // findEssentialMat/recoverPose 的 mask 标记内点，这里同步压缩两帧对应点。
    // mask 中非零值表示该匹配点符合估计出的极几何关系；外点通常来自误匹配、动态物体或遮挡。
    // 保留内点能让下一帧继续跟踪的特征更可靠，降低错误匹配累积。
    vector<Point2f> filtered_prev;
    vector<Point2f> filtered_curr;
    filtered_prev.reserve(prev_points.size());
    filtered_curr.reserve(curr_points.size());

    for (int i = 0; i < mask.rows && i < static_cast<int>(prev_points.size()) && i < static_cast<int>(curr_points.size()); ++i) {
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

    // triangulatePoints 输入的是归一化坐标，因此投影矩阵中不再乘相机内参 K。
    // 如果使用像素坐标，则投影矩阵需要写成 K[I|0] 和 K[R|t]；这里选择先归一化点。
    for (size_t i = 0; i < points_1.size(); ++i) {
        normalized_1.push_back(pixel2cam(points_1[i], camera_matrix));
        normalized_2.push_back(pixel2cam(points_2[i], camera_matrix));
    }

    Mat points_4d;
    triangulatePoints(pose_1, pose_2, normalized_1, normalized_2, points_4d);

    points.clear();
    for (int i = 0; i < points_4d.cols; ++i) {
        // 齐次坐标除以 w，得到三维欧式坐标。
        // points_4d 每一列是 [X, Y, Z, W]^T，归一化后保存为 Point3f。
        Mat column = points_4d.col(i);
        column /= column.at<float>(3, 0);
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
    const ProjectPaths paths = makeProjectPaths();
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

    // 命令行第一个参数可限制处理帧数，默认处理真值文件中的全部帧。
    // 例如运行 ./VO 200 时只处理前 200 帧，适合快速测试。
    const int requested_frames = argc > 1 ? atoi(argv[1]) : available_frames;
    const int max_frames = clampFrameCount(requested_frames, available_frames);
    if (max_frames < 2) {
        cerr << "Need at least two frames." << endl;
        return -1;
    }

    // 用前两帧完成初始特征跟踪和相对位姿估计。
    // 第 0 帧检测 FAST 特征，第 1 帧用 LK 光流找到对应点；
    // 有了两帧对应点后才能估计本质矩阵和初始相对位姿。
    Mat img_1 = readAndResizeImage(paths.left_image_pattern, 0, image_scale);
    Mat img_2 = readAndResizeImage(paths.left_image_pattern, 1, image_scale);
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
    // 主循环中 prev_image/prev_features 表示上一帧及其可继续跟踪的特征；
    // curr_image/curr_features 表示当前帧及上一帧特征跟踪过来的对应点。

    // raw_pose 保存当前累计世界位姿；rotation 存为逆向旋转以便把帧间位移累加到世界坐标。
    // recoverPose 得到的是“上一帧到当前帧”的相对运动，translation 表示当前坐标系下的方向。
    // 代码使用 -delta_translation 并乘以累计旋转，把相对平移方向转换到世界坐标后再累加。
    PoseRecord raw_pose;
    const double initial_scale = getAbsoluteScale(gt_positions, 1);
    raw_pose.rotation = toMatx33d(rotation.t());
    raw_pose.translation = (initial_scale > 0.0) ? initial_scale * toVec3d(translation) : toVec3d(translation);

    // trajectory 和 trajectory_frame_ids 一一对应，用于输出 TUM 轨迹、绘制估计轨迹并索引真值位置。
    // 初始估计来自第 0 到第 1 帧，所以第一条轨迹记录对应 frame_id=1。
    vector<PoseRecord> trajectory;
    trajectory.reserve(max_frames);
    vector<int> trajectory_frame_ids;
    trajectory_frame_ids.reserve(max_frames);
    trajectory.push_back(raw_pose);
    trajectory_frame_ids.push_back(1);

    future<Mat> next_frame_future;
    if (max_frames > 2) {
        // 异步预读下一帧，减少图像读取对主循环的阻塞。
        // 主线程处理当前帧特征和位姿时，后台线程提前把下一张图像读入内存。
        next_frame_future = async(std::launch::async, readAndResizeImage, paths.left_image_pattern, 2, image_scale);
    }

    for (int frame_id = 2; frame_id < max_frames; ++frame_id) {
        // 当前帧优先使用上一轮已经预读完成的结果。
        // 第 2 帧开始进入循环，因为第 0、1 帧已用于初始化。
        curr_image = next_frame_future.valid()
            ? next_frame_future.get()
            : readAndResizeImage(paths.left_image_pattern, frame_id, image_scale);
        if (frame_id + 1 < max_frames) {
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
        // 对上一帧保留下来的特征点进行光流跟踪，得到当前帧对应点。
        // featureTracking 会原地过滤 prev_features 和 curr_features，只留下通过检查的匹配。
        featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
        if (prev_features.size() < 8 || curr_features.size() < 8) {
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": not enough tracked features, re-detecting." << endl;
            }
            // 跟踪点过少时在上一帧重新检测特征，并再次尝试跟踪到当前帧。
            // 这样可以应对快速运动、模糊、纹理变化导致老特征大量丢失的情况。
            featureDetection(prev_image, prev_features, kTargetFeatureCount);
            curr_features.clear();
            featureTracking(prev_image, curr_image, prev_features, curr_features, tracking_status);
            if (prev_features.size() < 8 || curr_features.size() < 8) {
                // 重检测仍失败时跳过本帧位姿更新，避免用不可靠对应点污染轨迹。
                // 此时把 prev_image 更新为当前帧，相当于从当前帧重新开始积累可跟踪特征。
                prev_image = curr_image;
                prev_features = curr_features;
                if (shouldReportProgress(frame_id, max_frames)) {
                    cout << "Frame " << frame_id << "/" << (max_frames - 1)
                         << " tracked=0 status=reinit-failed" << endl;
                }
                continue;
            }
        }

        // 对当前帧和上一帧的匹配点重新估计本质矩阵。
        // 这里每一帧都重新用 RANSAC，是因为误匹配和动态物体会随时间变化。
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
            // RANSAC 内点过少说明本帧几何关系不可靠，只记录上一帧位姿并刷新特征。
            // 可能原因包括：图像纹理少、车辆转弯/运动过快、动态物体占比高、光流误匹配较多。
            // 这里不更新 raw_pose，但仍向 trajectory 写入当前 frame_id 对应的旧位姿，使输出帧序列连续。
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": pose inliers dropped to " << inlier_count << ", skipping update." << endl;
            }
            featureDetection(curr_image, curr_features, kTargetFeatureCount);
            trajectory.push_back(raw_pose);
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

        // 用真值相邻帧距离恢复单目尺度，再把当前帧间平移累加到世界坐标系。
        // 更新公式：
        //   raw_pose.translation += scale * raw_pose.rotation * (-delta_translation)
        //   raw_pose.rotation = R_relative^T * raw_pose.rotation
        // 其中 scale 来自真值，delta_translation 来自 recoverPose，只提供方向。
        const double scale = getAbsoluteScale(gt_positions, frame_id);
        if (scale > kMinTranslationScale) {
            raw_pose.translation += scale * (raw_pose.rotation * (-delta_translation));
            raw_pose.rotation = toMatx33d(rotation.t()) * raw_pose.rotation;
        } else if (verbose_logging) {
            // 相邻真值位移太小会让方向噪声占主导，跳过该帧位姿更新更稳。
            cout << "Frame " << frame_id << ": scale below threshold." << endl;
        }

        trajectory.push_back(raw_pose);
        trajectory_frame_ids.push_back(frame_id);
        // 只保留本质矩阵估计中的内点，下一轮继续跟踪更可靠的特征。
        // 注意 compact 后的 curr_features 会成为下一轮 prev_features。
        compactPointsByMask(prev_features, curr_features, mask);

        if (curr_features.size() < kMinTrackedFeatures) {
            // 特征点数量不足时在当前帧重新检测，保证后续帧有足够的跟踪点。
            // 重新检测会打断旧特征轨迹，但能防止特征数量持续下降导致后面无法估计位姿。
            if (verbose_logging) {
                cout << "Frame " << frame_id << ": tracked features down to " << curr_features.size() << ", re-detecting." << endl;
            }
            featureDetection(curr_image, curr_features, kTargetFeatureCount);
        }

        if (shouldReportProgress(frame_id, max_frames)) {
            // 进度信息包含当前保留的特征数、recoverPose 内点数和累计平移，便于判断运行质量。
            cout << "Frame " << frame_id << "/" << (max_frames - 1)
                 << " tracked=" << curr_features.size()
                 << " inliers=" << inlier_count
                 << " pose=(" << raw_pose.translation[0] << ", "
                 << raw_pose.translation[1] << ", "
                 << raw_pose.translation[2] << ")" << endl;
        }

        prev_image = curr_image;
        prev_features = curr_features;
        // 本轮结束后，当前帧成为下一轮的上一帧，当前帧特征成为下一轮要继续跟踪的特征。

        if (enable_visualization) {
            // 实时显示当前灰度图和 X-Z 平面轨迹。waitKey(1) 同时负责刷新 OpenCV 窗口事件。
            Mat trace = renderTraceCanvas(gt_positions, trajectory, trajectory_frame_ids, frame_id, max_frames);
            imshow("CAMERA IMAGE", curr_image);
            imshow("Trajectory Drawing", trace);
            waitKey(1);
        }
    }

    // 处理结束后保存估计轨迹文本，并生成最终轨迹对比图。
    // position.txt 使用 TUM 格式，可直接用 evo_ape/evo_rpe 评估；map2.png 中黑色为真值，绿色为 VO 估计。
    if (!ensureParentDirectory(paths.output_pose_path) || !ensureParentDirectory(paths.output_map_path)) {
        return -1;
    }
    ofstream output(paths.output_pose_path);
    if (!output.is_open()) {
        cerr << "Unable to open output file: " << paths.output_pose_path << endl;
        return -1;
    }
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const int frame_id = trajectory_frame_ids[i];
        if (frame_id >= 0 && frame_id < static_cast<int>(gt_timestamps.size())) {
            appendPose(output, gt_timestamps[frame_id], trajectory[i]);
        }
    }
    output.close();
    // 最后一张轨迹图即使在无 GUI 模式下也会保存，方便离线查看。
    Mat trace = renderTraceCanvas(gt_positions, trajectory, trajectory_frame_ids, max_frames - 1, max_frames);
    imwrite(paths.output_map_path, trace);
    cout << "Finished. Saved TUM trajectory to " << paths.output_pose_path
         << " and map to " << paths.output_map_path << endl;
    return 0;
}
