//
// Created by csh_i on 2024/4/18.
//

#ifndef CLION_TEST_HOLEHELPER_H
#define CLION_TEST_HOLEHELPER_H


//
// Created by csh_i on 2024/4/18.
//
#ifndef HOLE_HEPER_H
#define HOLE_HEPER_H
#include<opencv2/opencv.hpp>
#include<string>



struct HoleRes {
    // 状态信息
    bool status;
    bool inner_status;
    bool outer_status;
    //结果信息
    std::string msg; //错误信息
    cv::Mat img_drawed; // 绘制后的结果
    float offset_x;   // x位置偏差
    float offset_y;   // y位置偏差
    std::vector<float> inner_circle_res;
    std::vector<float> outer_circle_res;
    std::vector<int> rect_roi;
};



class HoleHelper {


public:
    // 缩放后图像的 宽高
    int resize_w;
    int resize_h;

    //获取内圆设定参数
    int offset; // 以内圆为中心,向外扩展的区域
    int min_inner_circle_radius;
    int max_inner_circle_radius;
    int min_inner_pixels_threshold;

    //获取外圆设定参数
    int min_outer_circle_radius;
    int max_outer_circle_radius;

    double scale=1.0;  // 现实距离与像素距离的比例

    HoleHelper();
    HoleRes get_holes_distance(cv::Mat img);

private:

    void detect_inner_circle(cv::Mat img, HoleRes& vis_res);

    void detect_outer_circle(cv::Mat img,  HoleRes& res);

    std::vector<int> detect_outer_circle_by_grad(cv::Mat img); // 通过梯度的策略获取外圆区域

    std::vector<int> kmeans_post_processing(cv::Mat img);

    std::vector<float> get_inner_circle_v2(cv::Mat img);

};








#endif // HOLE_HEPER_H

#endif //CLION_TEST_HOLEHELPER_H
