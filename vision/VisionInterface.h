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
#include "CameraCalibrationLine.h"

struct VisionResult
{
    bool status;
    stMeasureData stData;

    VisionResult &operator=(const VisionResult &other)
    {
        if (this != &other) // 防止自赋值
        {
            this->status = other.status;
            this->stData = other.stData;
        }
        return *this; // 返回当前对象的引用，以支持链式赋值
    }
};

class VisionInterface : public QThread
{
    Q_OBJECT
public:
    std::map<std::string, LineDetectRes> line_res; // LineDetectorRunner 返回结果,主要用于解析结果
    VisionResult vis_result;                       // VisionInterface 返回格式解析好的检测结果, 提供于其他线程使用
    VisionResult vis_result_;                      // 备份
    std::map<std::string, float> scales_line;      // 相机与现实世界的比例系数
    // 核心功能类
    LineDetectorRunner *line_handler = nullptr;
    LineDetector *line_helper = nullptr;
    CameraManager *camera_controls = nullptr;
    CameraCalibrationLine cameraCalib; // 相机标定

    // 线程相关
    QMutex mutex;
    SharedData *sharedDataLine = nullptr;
    bool isRunning = true;
    bool is_Detected = true;

    std::shared_ptr<spdlog::logger> logger;

    VisionInterface();
    ~VisionInterface() override;
    void run() override;

    void getDetectResult(DetectType type, stMeasureData *stm);
    std::map<std::string, LineDetectRes> VisionInterface::getLineRes();
    void parser_result(stMeasureData *stm);
    VisionResult getVisResult();
    void closeThread();
    std::map<std::string, CalibrationLine> getCameraCalibResults();
};

#endif // PDROOTV1_VISIONINTERFACE_H
