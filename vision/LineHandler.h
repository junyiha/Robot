#ifndef LINEHANDLER_H
#define LINEHANDLER_H

#include <string>
#include <iostream>
#include <time.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <QLabel>
#include <QObject>
#include <QThread>
#include <QMatrix>
#include <QApplication>
#include<QMutex>
#include "LineHelper.h"
#include "CameraControls.h"
#include <QWaitCondition>
#include "../vision/ShareData.h"



// 视觉检测结果
struct LineVisionRes{
    bool status;
    float dist;
    cv::Mat img_drawed;
    std::string cam_name;
};



class LineHandler: public QThread
{
//  Q_OBJECT
public:
    
    //线程控制标志
    bool detect_control_flag = false;
    bool thread_control_flag = true;
    //结果保存
    std::map<std::string, LineVisionRes> results;
    //硬件设备类
    LineHelper* line_helper= nullptr;
    CameraControls* cam_controls = nullptr;
    SharedData* sharedData = nullptr;

    void run();
    QMutex data_lock;
    QWaitCondition condition;
    void is_start_detect(bool flag);
    void closed();
    LineHandler(LineHelper* hole,CameraControls* cam, SharedData* shared);
    ~LineHandler();  



};



#endif // LINEHANDLER_H
