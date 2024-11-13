//
// Created by csh_i on 2024/6/25.
//

#ifndef HK_DEMO2_PARAMETERS_H
#define HK_DEMO2_PARAMETERS_H
#include"MvCamera.h"
#include <iostream>

// 海康相机设备连接结构体
struct HKCameraDeviceInfo{
    const char* camName ;                                         //  相机名称
    const char* camIp;                                            //  相机IP地址
    const char*  ethIp ;                                             //  以太网IP地址
    MV_CC_DEVICE_INFO pstDeviceInfo ;         //  相机设备信息相关
    MV_CC_DEVICE_INFO stDevInfo = {0};
    MV_GIGE_DEVICE_INFO stGigEDev = {0};
};


enum DetectType{
    LINE, HOLE, LIDAR
};


// LineDetectorRunner 类中, 直线检测结果结构体
struct LineDetectRes{
    bool status;
    float dist;
    cv::Mat img_drawed;
    std::string cam_name;
};


// LineDetector中, 直线及直线检测结果的结构体定义
struct MLine{
    float x1;
    float y1;
    float x2;
    float y2;
    float slope;
};


struct LineResult {
    float lineDist;
    bool status;
    bool inkLineStatus;
    bool referLineStatus;

    std::string errorInfo; //错误信息
    MLine inkResult;  // 墨线结果
    MLine refResult;  // 参考线结果
    cv::Mat imgDrawed;
};











#endif //HK_DEMO2_PARAMETERS_H
