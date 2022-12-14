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
        /// @param trans_mtx - perspective to bird's eye view transform
        /// @param gab_mtx  - gabor filter kernel
        void set_params(const cv::Mat &trans_mtx,const cv::Mat &gab_mtx);

        /// @brief 
        /// @param image - input image in bird's eyeview
        /// @param img_out 
        void filter_image(const cv::Mat &image,cv::Mat &img_out);
        
        /// @brief 
        /// @param pix_loc - 2D pixel coordinates
        void get_pixels(std::vector<cv::Point2i> &pix_loc);

        /// @brief 
        /// @param img 
        void subtract_average_intensity(cv::Mat &img);

        // /// @brief 
        // /// @param gaus_kern -Gaussian kernel
        // /// @param img_gbr - Gabor filtered image
        // /// @param img_gau  - Gaussian filtered image
        // void get_gauss_filtered(const cv::Mat &gaus_kern,cv::Mat &img_gbr,cv::Mat &img_gau);

    private:
        cv::Mat img_warped_;
        cv::Mat img_gabor_;
        cv::Mat img_out_;
        std::vector<cv::Point2i> indices_;
        std::vector<cv::Point2i> m_indices_;
        std::vector<cv::Point2i> f_indices_;
        cv::Mat gabor_kernel_;
        cv::Mat trans_matrix_;

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