#pragma once
#pragma once
#include<string>
#include<iostream>
#include<vector>
#include "mlsd.h"
#include <opencv2/opencv.hpp>
#include "utils.h"


//定义结果的数据结构

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


struct InkInfo {   
	bool is_upper;  
	int base_y;
};


class LineHelper {

public:

	LineHelper();
	//检测两条直线并计算垂直距离
	LineRes compute_lines_distance(cv::Mat img);
	MLSD line_tooler;


private:
	unsigned int resize_h;  
	unsigned int resize_w;

	//线段目标图像分布的区间大小
	unsigned int min_pos_valid; 
	unsigned int max_pos_valid; 

	// 板线与墨迹之间最小,最大距离
	unsigned int min_dist_lines; 
	unsigned int max_dist_lines; 

	// 板线与墨迹线之间最大角度偏差
	unsigned int max_angle_degrees;

	unsigned int min_len_seg; //线段的最小长度

    float scale = 1.0;  // 现实距离与像素距离的比例


	//获取板界线 
    void get_border_line(cv::Mat img_gray, cv::Mat bin_img, std::vector<std::vector<float>> pred_lines, LineRes& res);

	//获取墨迹线
    void get_ink_line(cv::Mat img, std::vector<std::vector<float>> pred_lines,LineRes& res);

	// LSD直线检测 
    std::vector<cv::Vec4f> line_detect_lsd(cv::Mat bin_img);

	// 图像预处理
    void img_processing(cv::Mat img, cv::Mat& img_gray, cv::Mat& bin_img);


    InkInfo get_ink_rough_property(cv::Mat bin_img,  std::vector<float> line);

};
