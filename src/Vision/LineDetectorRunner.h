//
// Created by chen on 2024/7/6.
//

#ifndef PDROOTV1_LINEDETECTORRUNNER_H
#define PDROOTV1_LINEDETECTORRUNNER_H
#include <QThread>
#include <QMutex>
#include "LineDetector.h"
#include <QWaitCondition>
#include "ShareData.h"
#include "CameraManager.h"
#include <spdlog/logger.h>
#include "Parameters.h"

class LineDetectorRunner : public QThread
{
    Q_OBJECT
public:
    // 线程控制标志
    bool detect_control_flag = true; // 是否进行检测的标志
    bool thread_control_flag = true;

    std::shared_ptr<spdlog::logger> logger;

    LineDetectorRunner(LineDetector *line_helper, CameraManager *cam_controls, SharedData *shared);
    ~LineDetectorRunner();
    void run() override;
    bool is_running = false;

    // 结果保存
    std::map<std::string, LineDetectRes> results;
    // 硬件设备类
    LineDetector *line_helper = nullptr;
    CameraManager *cam_controls = nullptr;
    SharedData *sharedData = nullptr;

    QMutex data_lock;
    QWaitCondition condition;
    void is_start_detect(bool flag);
    void closedThread();
    void pauseDetect();
    void restartDetect();
};

#endif // PDROOTV1_LINEDETECTORRUNNER_H
