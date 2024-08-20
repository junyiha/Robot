#pragma once
#pragma once
#include<string>
#include<iostream>
#include<vector>
#include "mlsd.h"
#include <opencv2/opencv.hpp>
#include "utils.h"


//�����������ݽṹ

struct LineRes {

	float dist;
	bool status; 
	bool ink_line_status;
	bool border_line_status;
    std::string error_info; //������Ϣ
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
	//�������ֱ�߲����㴹ֱ����
	LineRes compute_lines_distance(cv::Mat img);
	MLSD line_tooler;


private:
	unsigned int resize_h;  
	unsigned int resize_w;

	//�߶�Ŀ��ͼ��ֲ��������С
	unsigned int min_pos_valid; 
	unsigned int max_pos_valid; 

	// ������ī��֮����С,������
	unsigned int min_dist_lines; 
	unsigned int max_dist_lines; 

	// ������ī����֮�����Ƕ�ƫ��
	unsigned int max_angle_degrees;

	unsigned int min_len_seg; //�߶ε���С����

    float scale = 1.0;  // ��ʵ���������ؾ���ı���


	//��ȡ����� 
    void get_border_line(cv::Mat img_gray, cv::Mat bin_img, std::vector<std::vector<float>> pred_lines, LineRes& res);

	//��ȡī����
    void get_ink_line(cv::Mat img, std::vector<std::vector<float>> pred_lines,LineRes& res);

	// LSDֱ�߼�� 
    std::vector<cv::Vec4f> line_detect_lsd(cv::Mat bin_img);

	// ͼ��Ԥ����
    void img_processing(cv::Mat img, cv::Mat& img_gray, cv::Mat& bin_img);


    InkInfo get_ink_rough_property(cv::Mat bin_img,  std::vector<float> line);

};
