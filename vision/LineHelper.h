//
// Created by csh_i on 2024/6/12.
//

#ifndef CMAKE_TEST_LINEHELPER_H
#define CMAKE_TEST_LINEHELPER_H
#include "mlsd.h"
#include <opencv2/opencv.hpp>
#include "utils.h"


struct LineRes {
    float dist;
    bool status;
    bool ink_line_status;
    bool border_line_status;
    std::string error_info; //错误信息
    std::vector<float> ink_res;
    std::vector<float> border_res;
    cv::Mat img_drawed;
};


class LineHelper {
public:

    LineHelper() ;

    // 图像预处理
    void img_processing(cv::Mat img, cv::Mat& img_gray, cv::Mat& bin_img);

    LineRes compute_lines_distance(cv::Mat img, bool is_upper=false);

    std::vector<std::vector<float>>  get_all_possible_lines(cv::Mat img);

    std::vector<float>  get_border_line(cv::Mat img, std::vector<std::vector<float>> lines);
    std::vector<float> get_border_line_by_lsd(cv::Mat img);

    //获取墨迹线
    std::vector<float>  get_ink_line(cv::Mat img, std::vector<float> &border_line, std::vector<std::vector<float>> &pred_lines);




private:

    MLSD line_detector;

    unsigned  int  resize_h;
    unsigned  int  resize_w;
    unsigned  int  min_len_seg;

    unsigned  int min_pos_valid;
    unsigned  int max_pos_valid ;

    unsigned  int min_dist_lines ;
    unsigned  int max_dist_lines ;
    unsigned  int max_angle_degrees ;


    void imageLocalBinarization(cv::Mat img, const std::vector<std::vector<float>> &lines, cv::Mat &bin_img);
};


#endif //CMAKE_TEST_LINEHELPER_H
