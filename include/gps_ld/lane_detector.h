
#ifndef LANEDETECTOR_H_
#define LANEDETECTOR_H_

#include <ros/ros.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/Inspva.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// #include <fstream>
// #include <chrono>
#include "lanemarking_extractor.h"
#include "helpers.h"

class LaneDetector{    
    public:          
        /// @brief 
        LaneDetector();
        /// @brief 
        ~LaneDetector();
        /// @brief 
        /// @param img_msg 
        void image_callback(const sensor_msgs::CompressedImageConstPtr& img_msg);

        /// @brief 
        /// @param imu 
        void imu_callback(const sensor_msgs::Imu::ConstPtr& imu);

        /// @brief 
        /// @param msg 
        void gps_callback2(const novatel_gps_msgs::Inspva msg);

        /// @brief 
        /// @param msg 
        void gps_callback(const novatel_gps_msgs::NovatelPosition msg);

        /// @brief 
        void update_lane();

        /// @brief 
        /// @param pts 
        /// @param x 
        /// @param y 
        void gps_to_image_coordinate(std::vector<GpsPt>&pts,std::vector<int>& x,std::vector<int> &y);

        /// @brief 
        /// @param img 
        void plot_to_image(cv::Mat &img);

        /// @brief 
        void calculate_index();

        /// @brief 
        /// @param pts 
        void interpolate_points(std::vector<GpsPt> &pts);

    private:
        // ROS node handle and image transport objects
        ros::NodeHandle nh_;    
        ros::Subscriber image_sub_;  
        ros::Subscriber gps_sub_;
        ros::Subscriber imu_sub_;   

        int img_w_;
        int img_h_;
        int MAX_PTS = 30;

        cv::Mat image_;
        cv::Mat img_org_;
        cv::Mat img_warped_;
        cv::Mat img_filtered_;
        cv::Mat trans_matrix_;
        cv::Mat inv_trans_matrix_;
        cv::Mat gabor_kernel_;
        ///
        cv::Mat gauss_kernel_;
        ///
        bool initialized_;
               
        /// @brief 
        void initialize_params();   

        /// @brief 
        /// @param ref_pt 
        /// @param er_delta 
        /// @param anchors 
        void get_anchor_points(const int ref_pt,const int er_delta,std::vector<int> &anchors);

        /// @brief 
        void evaluate_lines();//const std::vector<Eigen::VectorXd> &lines,Eigen::VectorXd &sel_line);

        /// @brief
        void get_pixels(const Eigen::Vector3d &ref,std::vector<cv::Point> &pix_out);
};
#endif //LANEDETECTOR_H_
