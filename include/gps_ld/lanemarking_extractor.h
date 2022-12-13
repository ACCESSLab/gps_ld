#ifndef LANEMARKINGEXTRACTOR_H_
#define LANEMARKINGEXTRACTOR_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
// #include <iostream>

class LaneMarkingExtractor{       

    public:
        /// @brief 
        LaneMarkingExtractor();
        /// @brief 
        ~LaneMarkingExtractor();
        /// @brief 
        /// @param trans_mtx 
        /// @param gab_mtx 
        void set_params(const cv::Mat &trans_mtx,const cv::Mat &gab_mtx);
        /// @brief 
        /// @param image
        /// @param img_out 
        void filter_image(cv::Mat &image,cv::Mat &img_out);
        
        /// @brief 
        /// @param pix_loc 
        void get_pixels(std::vector<cv::Point2i> &pix_loc);
        /// @brief 
        /// @param img 
        void subtract_average_intensity(cv::Mat &img);
        /// @brief 
        /// @param gaus_kern 
        /// @param img_gbr 
        /// @param img_gau 
        void get_gauss_filtered(cv::Mat &gaus_kern,cv::Mat &img_gbr,cv::Mat &img_gau);

    private:
        cv::Mat img_warped_;
        cv::Mat img_gabor_;
        cv::Mat img_out_;
        std::vector<cv::Point2i> indices_;
        std::vector<cv::Point2i> m_indices_;
        std::vector<cv::Point2i> f_indices_;
        cv::Mat gabor_kernel_;
        cv::Mat trans_matrix_;
        //bool img_set_;
        cv::Mat img_tmp_;

        const int INTENSITY_TH = 30; //pixels
        const int AVG_INTENSITY_TH = 80; //pixels
        const int LTH_MIN = 2; //lane marking min thickess
        const int LTH_MAX = 16; //lane marking max thickness

        /// @brief 
        void preprocess();
        /// @brief 
        void apply_image_filtering();
        /// @brief 
        void filter_image();
        /// @brief 
        void zero_indices();
        /// @brief 
        void select_indices();
        /// @brief 
        void final_indices();
};
#endif //LANEMARKINGEXTRACTOR_H_