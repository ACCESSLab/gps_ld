#include "lane_detector.h"

LaneDetector::LaneDetector()
{
}

LaneDetector::~LaneDetector()
{
}

void LaneDetector::image_callback(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
}

void LaneDetector::imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
}

void LaneDetector::update_lane()
{
}

void LaneDetector::gps_to_image_coordinate(std::vector<GpsPt> &pts, std::vector<int> &x, std::vector<int> &y)
{
}

void LaneDetector::plot_to_image(cv::Mat &img)
{
}

void LaneDetector::calculate_index()
{
}

void LaneDetector::interpolate_points(std::vector<GpsPt> &pts)
{
}

void LaneDetector::initialize_params()
{
}

void LaneDetector::get_anchor_points(const int ref_pt, const int er_delta, std::vector<int> &anchors)
{
}

void LaneDetector::evaluate_lines()
{
}

void LaneDetector::get_pixels(const Eigen::Vector3d &ref, std::vector<cv::Point> &pix_out)
{
}
