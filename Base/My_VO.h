#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <fstream>
#include <filesystem>
#include <string>
#include <vector>

// 针孔相机内参。
// fx/fy 表示焦距，cx/cy 表示主点坐标，单位都是像素。
// 这些参数用于构造相机矩阵 K，并参与本质矩阵、三角化和 PnP 计算。
struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

// 工程运行所需的输入输出路径集合。
// 输入路径包括 KITTI 图像序列、相机配置和真值轨迹；
// 输出路径包括最终轨迹、纯几何轨迹和轨迹对比图。
struct ProjectPaths {
    std::string project_root;
    std::string camera_info_path;
    std::string left_image_pattern;
    std::string gt_pose_path;
    std::string output_pose_path;
    std::string output_geometry_pose_path;
    std::string output_map_path;
};

// 位姿记录，表示相机在世界坐标系下的姿态。
// rotation 是相机坐标到世界坐标的旋转 R_cw；
// translation 是相机中心在世界坐标系中的位置 t_cw。
struct PoseRecord {
    cv::Matx33d rotation = cv::Matx33d::eye();
    cv::Vec3d translation = cv::Vec3d(0.0, 0.0, 0.0);
};

// 读取相机内参配置文件，成功时填充 intrinsics 并返回 true。
bool loadCameraIntrinsics(const std::string& camera_info_path, CameraIntrinsics& intrinsics);
// 从 TUM 真值轨迹中提取每帧平移位置，用于尺度恢复和轨迹绘图。
std::vector<cv::Vec3d> loadGroundTruthPositions(const std::string& pose_path);
// 从 TUM 真值轨迹中提取时间戳，用于按 TUM 格式输出估计轨迹。
std::vector<double> loadGroundTruthTimestamps(const std::string& pose_path);
// 根据 fx/fy/cx/cy 构造 OpenCV 使用的 3x3 相机矩阵。
cv::Mat buildCameraMatrix(const CameraIntrinsics& intrinsics);
// 图像缩放后同步缩放相机内参，保证像素坐标模型一致。
void scaleCameraIntrinsics(CameraIntrinsics& intrinsics, double scale);
// 按帧号读取灰度图；读取失败时返回空 Mat。
cv::Mat readImageOrEmpty(const std::string& image_pattern, int frame_id);
// 将图像路径模板中的帧号格式化为实际文件路径。
std::string formatImagePath(const std::string& image_pattern, int frame_id);

// 使用 FAST 检测特征点，并按响应强度保留最多 max_corners 个点。
void featureDetection(const cv::Mat& image, std::vector<cv::Point2f>& points, int max_corners);
// 使用 LK 光流跟踪两帧特征，并做前后向一致性过滤，返回保留点数量。
int featureTracking(
    const cv::Mat& prev_image,
    const cv::Mat& curr_image,
    std::vector<cv::Point2f>& prev_points,
    std::vector<cv::Point2f>& curr_points,
    std::vector<uchar>& status);
// 根据 RANSAC/recoverPose 输出的 mask 同步过滤两帧匹配点。
void compactPointsByMask(std::vector<cv::Point2f>& prev_points, std::vector<cv::Point2f>& curr_points, const cv::Mat& mask);

// 将像素坐标转换为归一化相机坐标。
cv::Point2f pixel2cam(const cv::Point2d& pixel, const cv::Mat& camera_matrix);
// 根据两帧匹配点和相对位姿三角化 3D 点。
void triangulation(
    const std::vector<cv::Point2f>& points_1,
    const std::vector<cv::Point2f>& points_2,
    const cv::Mat& camera_matrix,
    const cv::Mat& rotation,
    const cv::Mat& translation,
    std::vector<cv::Point3f>& points);

// 读取真值相邻帧距离，作为单目 VO 的绝对尺度。
double getAbsoluteScale(const std::vector<cv::Vec3d>& gt_positions, int frame_id);

// 按 TUM 格式写出一帧估计位姿：timestamp tx ty tz qx qy qz qw。
void appendPose(std::ofstream& output, double timestamp, const PoseRecord& pose);
// 自动判断当前环境是否适合打开 OpenCV 显示窗口。
bool isVisualizationEnabled();
// 判断是否启用详细日志。
bool isVerboseLoggingEnabled();
// 获取图像缩放比例，默认不缩放。
double getImageScale();
