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

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

struct ProjectPaths {
    std::string camera_info_path;
    std::string left_image_pattern;
    std::string gt_pose_path;
    std::string output_pose_path;
};

struct PoseRecord {
    cv::Mat rotation;
    cv::Mat translation;
};

bool loadCameraIntrinsics(const std::string& camera_info_path, CameraIntrinsics& intrinsics);
std::vector<cv::Vec3d> loadGroundTruthPositions(const std::string& pose_path);
cv::Mat buildCameraMatrix(const CameraIntrinsics& intrinsics);
cv::Mat readImageOrEmpty(const std::string& image_pattern, int frame_id);
std::string formatImagePath(const std::string& image_pattern, int frame_id);

void featureDetection(const cv::Mat& image, std::vector<cv::Point2f>& points);
void featureTracking(
    const cv::Mat& prev_image,
    const cv::Mat& curr_image,
    std::vector<cv::Point2f>& prev_points,
    std::vector<cv::Point2f>& curr_points,
    std::vector<uchar>& status);

cv::Point2f pixel2cam(const cv::Point2d& pixel, const cv::Mat& camera_matrix);
void triangulation(
    const std::vector<cv::Point2f>& points_1,
    const std::vector<cv::Point2f>& points_2,
    const cv::Mat& camera_matrix,
    const cv::Mat& rotation,
    const cv::Mat& translation,
    std::vector<cv::Point3f>& points);

double getAbsoluteScale(const std::vector<cv::Vec3d>& gt_positions, int frame_id);

bool isForwardMotion(const cv::Mat& translation);
void appendPose(std::ofstream& output, const cv::Mat& translation);
bool isVisualizationEnabled();
