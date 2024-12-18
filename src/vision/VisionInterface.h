//
// Created by chen on 2024/7/6.
//

#ifndef PDROOTV1_VISIONINTERFACE_H
#define PDROOTV1_VISIONINTERFACE_H
#include <QThread>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "CameraManager.h"
#include "LineDetector.h"
#include "LineDetectorRunner.h"
#include "ShareData.h"
#include "../Task/Measure.h"
#include <QMutex>
#include "Parameters.h"
#include "LidarHandler.h"





struct VisionResult {
    bool lineStatus;
    bool laserStatus;
    stMeasureData stData;

    VisionResult& operator=(const VisionResult& other)
    {
        if (this != &other) // 防止自赋值
        {
            this->lineStatus = other.lineStatus;
            this->laserStatus = other.laserStatus;
            this->stData = other.stData;
        }
        return *this; // 返回当前对象的引用，以支持链式赋值
    }
};





class VisionInterface : public QThread {
    Q_OBJECT
public:

    std::map<std::string, LineDetectRes> line_res; // LineDetectorRunner 返回结果,主要用于解析结果
    std::map<std::string, LidarData> lidar_res;
    VisionResult vis_result;  // VisionInterface 返回格式解析好的检测结果, 提供于其他线程使用
    VisionResult vis_result_;  // 备份
    std::map<std::string, float> scales_line;  //相机与现实世界的比例系数
    std::map<std::string, float> camera_offset;  //相机与现实世界的比例系数

    //核心功能类
    LineDetectorRunner* line_handler = nullptr;
    LineDetector* line_helper = nullptr;
    CameraManager* camera_controls = nullptr;

    //轮廓激光传感器
    LidarHelper* lidar_helper = nullptr;
    LaserScanerControls* laser_controls = nullptr;
    LidarHandler* lidar_handler = nullptr;
    int layer2camera[4] = {
            4,5, 1, 3
    };  // 轮廓激光映射到对应相机



    // 线程相关
    QMutex mutex;
    QMutex imagesMutex;
    QMutex pointMaskMutex;
    SharedData* sharedDataLine = nullptr;
    SharedData* sharedDataLaser = nullptr;
    bool isRunning = true;
    bool is_Detected = true;

    std::shared_ptr<spdlog::logger> logger;

    VisionInterface();
    ~VisionInterface() override;
    void run() override;

    void getDetectResult(VisionResult& result);
    std::map<std::string, LineDetectRes> VisionInterface::getLineRes();
    void parser_result(std::string paserType, stMeasureData* stm);
    VisionResult getVisResult();
    void closeThread();

};


#endif //PDROOTV1_VISIONINTERFACE_H
