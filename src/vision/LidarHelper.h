//
// Created by csh_i on 2024/4/28.
//

#ifndef CLION_ONNX_LIDARHELPER_H
#define CLION_ONNX_LIDARHELPER_H

#ifndef DETECT_H
#define DETECT_H

#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<vector>
#include<time.h>
#define M_PI 3.14159265358979323846
#include "utils.h"



// 雷达数据结构体
struct LidarData{
    float dist;
    float angle;
    float gap;
    bool flag=0;//是否有效
    cv::Mat show;
    std::string error;
    bool  pointValidFlag = false;  // 边缘点位置有效性
    cv::Point2f board;
};


class LidarHelper
{
public:
    LidarHelper();

    // 检测结果
    bool resultFlag=false;
    float lidarAngle=0;
    float lidarDist=0;
    float gap=0;
    cv::Mat show;
    std::string error;
    bool Left;
    bool vertexValidFlag = false;  // 绝缘板获取位置有效性    (自动取板功能使用)
    cv::Point2f vertex; // 绝缘板边缘点位置


    //检测相关函数
    void lidarDetecter(cv::Mat img,bool isleft,bool revAngle=false,int min_X=0, int min_Y=0, bool pIsLeft=false); // 雷达检测函数
    float getAngle(cv::Point2f p1,cv::Point2f p2);  // 获取两个点的角度

    void clear();   // 清空所有检测结果
    void shrink(); // 缩小图片

    float getGap(std::vector<cv::Point2f> line1,std::vector<cv::Point2f> line2);  // 获取两个直线之间的距离
    cv::Mat showImg(std::vector<cv::Point2f> board,std::vector<cv::Point2f> ref); // 显示检测结果
    void filterBorderAndReferenceLine(const cv::Mat &img, bool pIsLeft,
                                 const std::vector<std::vector<cv::Point2f>> &linesEnd,
                                 std::vector<cv::Point2f> &borderLine,
                                 std::vector<cv::Point2f> &referLine);
};



#endif // DETECT_H
#endif //CLION_ONNX_LIDARHELPER_H
