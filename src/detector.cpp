#include "detector.h"

Detector::Detector()
{
}

Detector::~Detector()
{
}

void Detector::image_callback(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
}

void Detector::imu_callback(const sensor_msgs::Imu::ConstPtr &imu)
{
}

void Detector::update_lane()
{
}

void Detector::gps_to_image_coordinate(std::vector<ExtendedWaypoint> &pts, std::vector<int> &x, std::vector<int> &y)
{
}

void Detector::plot_to_image(cv::Mat &img)
{
}

void Detector::calculate_index()
{
}

void Detector::interpolate_points(std::vector<ExtendedWaypoint> &pts)
{
}

void Detector::initialize_params()
{
}

void Detector::get_anchor_points(const int ref_pt, const int er_delta, std::vector<int> &anchors)
{
}

void Detector::evaluate_lines()
{
}

void Detector::get_pixels(const Eigen::Vector3d &ref, std::vector<cv::Point> &pix_out)
{
}
