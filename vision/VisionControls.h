//
// Created by csh_i on 2024/4/28.
//

#ifndef CLION_TEST_VISIONCONTROLS_H
#define CLION_TEST_VISIONCONTROLS_H
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "HoleHelper.h"
#include "CameraControls.h"
#include "LineHelper.h"
#include "GocatorControls.h"
#include "LidarHelper.h"
#include "LidarHandler.h"
#include "HoleHandler.h"
#include "LineHandler.h"
#include <QMutex>
#include <QWaitCondition>
#include "../vision/ShareData.h"
#include <Eigen/Dense>
#include "../Task/Measure.h"
#include "Parameters.h"



// 视觉检测结果
struct VisionRes{
    bool status;
    float dist;
    cv::Mat img_drawed;
    std::string cam_name;
};




class VisionControls {
public:
    //variables
    std::map<std::string, HoleVisionRes> hole_res;
    std::map<std::string, LineVisionRes> line_res;
    std::map<std::string, LidarData> lidar_res;
    DetectType detect_type;


    // 相机操控对象 边缘相机, 圆孔相机, 红外相机
    CameraControls* camera_controls_edge = nullptr;
    CameraControls* camera_controls_hole = nullptr;
    GocatorControls* gocator_controls = nullptr;

    //比例系数
    std::map<std::string, float> scales_hole;
    std::map<std::string, float> scales_line;


    // HoleHelper 圆孔检测对象
    HoleHelper* hole_helper = nullptr;
    // LineHelper 直线检测对象
    LineHelper* line_helper = nullptr;
    //  雷达检测对象
    LidarHelper* lidar_helper = nullptr;

    //处理线程
    LidarHandler* lidar_handler = nullptr;
    HoleHandler*  hole_handler = nullptr;
    LineHandler*  line_handler = nullptr;


    SharedData* sharedDataHole=nullptr;
    SharedData* sharedDataLine=nullptr;
    SharedData* sharedDataGocator=nullptr;


    VisionControls();
    ~VisionControls();
    /**
     * @brief 启动指定类型的检测任务
     * 
     * @param type 检测类型
     */
    void enableDetection(DetectType type);
    /**
     * @brief 获取检测结果
     * 
     * @param type 检测类型
     * @param stm 检测结果
     */
    void getDetectResult(DetectType type,stMeasureData* stm);
    void parser_result(stMeasureData* stm);
};







#endif //CLION_TEST_VISIONCONTROLS_H
