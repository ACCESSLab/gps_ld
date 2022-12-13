
#include "lanemarking_extractor.h"

LaneMarkingExtractor::LaneMarkingExtractor(){
    
}
LaneMarkingExtractor::~LaneMarkingExtractor(){

}

void LaneMarkingExtractor::get_pixels(std::vector<cv::Point2i> &pix_loc){    
    pix_loc = f_indices_;
}

void LaneMarkingExtractor::set_params(const cv::Mat &trans_mtx,const cv::Mat &gab_mtx){
    trans_matrix_ = trans_mtx;
    gabor_kernel_ = gab_mtx;
}

void LaneMarkingExtractor::filter_image(cv::Mat &img_wrp,cv::Mat &img_out){
    img_warped_ = img_wrp;
    preprocess();
    // img_out = img_gabor_;
    apply_image_filtering();
    img_out = img_out_;
}

void LaneMarkingExtractor::get_gauss_filtered(cv::Mat &gaus_kern,cv::Mat &img_gbr,cv::Mat &img_gau){
    cv::filter2D(img_tmp_,img_gau,CV_8U,gaus_kern);
    img_gbr = img_gabor_.clone();
}

void LaneMarkingExtractor::preprocess(){
    //std::cout<<"img:"<<image_.dims<<",ch:"<<image_.channels()<<"\n";
    //cv::Mat img_warped;
    //cv::warpPerspective(image_, img_warped, trans_matrix_, image_.size());

    //convert image to YCrCb color space
    cv::Mat img_ycrcb;            
    cv::cvtColor(img_warped_,img_ycrcb,cv::COLOR_BGR2YCrCb);
    //cv::imshow("ycrcb",img_ycrcb);
    cv::Mat channels[3];
    //Separate the cr channel: which contains the yellow line information
    cv::split(img_ycrcb,channels);  
    cv::Mat img_gray = channels[0];// ConvertToGrayImage(img_warped);  
    cv::GaussianBlur(img_gray,img_gray,cv::Size(5,5),3);
    //subtract_average_intensity2(img_gray);
    subtract_average_intensity(img_gray);
    img_tmp_ = img_gray.clone();
    cv::Mat gabor_gray;
    cv::filter2D(img_gray,gabor_gray,CV_8U,gabor_kernel_);

    cv::Mat img_cb;// = channels[2];        
    cv::subtract(channels[1],channels[2],img_cb); //cr-cb = img_cb         
     
    cv::filter2D(img_cb,img_cb,CV_8U,gabor_kernel_);        

    // //Combine the Cr channel with the grayscale image      
    cv::addWeighted(img_cb, 1.0, gabor_gray, 1.0, 0.0, img_gabor_);

    //ApplyImageFiltering();//img_filtered_ will be available
}

void LaneMarkingExtractor::apply_image_filtering(){
    
    zero_indices();
    if(indices_.size()>1){ 
        select_indices();
        if(m_indices_.size()>1){
            final_indices();
            filter_image();
        }
    }
}
void LaneMarkingExtractor::filter_image(){ 
    img_out_ = cv::Mat::zeros(img_gabor_.rows, img_gabor_.cols, img_gabor_.type());
    cv::Point2i p1,p2;
    if (f_indices_.size() > 2){
        for (int q = 0; q < f_indices_.size() - 1;) {
            p1 = f_indices_[q];
            p2 = f_indices_[q + 1];
            if (p1.x == p2.x){
                for (int k = p1.y; k < p2.y; k++){
                    img_out_.at<uchar>(p1.x, k) = img_gabor_.at<uchar>(p1.x, k);
                }
                q += 2;
            }else{
                q++;
            }
        }
    }      
}
void LaneMarkingExtractor::zero_indices(){

    indices_.clear();
    static int i_start = 0;
    static int i_end = img_gabor_.rows;    
    for (int i = 0; i < img_gabor_.rows; i++){       
        for (int j = 0; j <img_gabor_.cols; j++) {
            if (img_gabor_.at<uchar>(i, j) < INTENSITY_TH){
                indices_.emplace_back(cv::Point2i(i, j));
            }
        }
    }

}

void LaneMarkingExtractor::select_indices(){
    m_indices_.clear();
    cv::Point2i p1, p2;            
    int dist;
    int n = 0;
    for (n = 0; n < indices_.size() - 1;){
        p1 = indices_[n];
        p2 = indices_[n + 1];
        if (p1.x == p2.x){
            dist = p2.y - p1.y;
            if (dist > LTH_MIN && dist < LTH_MAX){
                m_indices_.emplace_back(p1);
                m_indices_.emplace_back(p2);
                n += 2;
            }else{
                n++;
            }
        }else{
            n++;
        }
    }
}

void LaneMarkingExtractor::final_indices(){
    f_indices_.clear();
    int avg_intensity = 0;
    int count = 0;
    int j = 0;
    cv::Point2i p1, p2;
    //std::vector<cv::Point2i> final_indices;
    for (j = 0; j < indices_.size() - 1;) {
        p1 = indices_[j];
        p2 = indices_[j + 1];
        if (p1.x == p2.x){
            int dist = p2.y - p1.y;
            if (dist > LTH_MIN && dist < LTH_MAX){
                for (int i = p1.y; i < p2.y; i++){
                    avg_intensity += img_gabor_.at<uchar>(p1.x, i);
                    count++;
                }
            }
        }
        if (count != 0){
            avg_intensity = int(avg_intensity / count);
            if (avg_intensity > AVG_INTENSITY_TH) {
                f_indices_.push_back(p1);
                f_indices_.push_back(p2);
                j += 2;
            }else{
                j++;
            }
        }else{
            j++;
        }
        avg_intensity = 0;
        count = 0;
    }
}

void LaneMarkingExtractor::subtract_average_intensity(cv::Mat &img){
    int width = img.cols;
    int height = img.rows;

    
    int win_w = width;
    int win_h = 20;
    int r_start = height-1 - win_h;
    int c_start = 0;

    int avg_intensity = 0;
    bool valid = true;
    while(valid){        

        cv::Mat region = cv::Mat(img,cv::Rect(c_start,r_start,win_w,win_h));
        auto sum = cv::sum(region);
        avg_intensity = int(sum[0]/(win_h*win_w));
       
        for(int i=r_start;i<(r_start+win_h);++i){
            for(int j=(c_start);j<(c_start+win_w);++j){  
                (img.at<uchar>(i,j)>avg_intensity)?
                    img.at<uchar>(i,j) -= avg_intensity:
                    img.at<uchar>(i,j) = 0;
            }
        }
        r_start -= win_h;
        avg_intensity = 0;
        if(r_start<(0)) 
            valid = false;
    }    
}
