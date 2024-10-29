
#ifndef PDROBOT_LIDARHANDLER_H
#define PDROBOT_LIDARHANDLER_H
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
#include "LidarHelper.h"
#include "LaserScanerControls.h"
#include "ShareData.h"





class LidarHandler: public QThread
{
//  Q_OBJECT
public:


    bool detect_control_flag = true;
    bool thread_control_flag = true;
    std::map<std::string, LidarData> results;
    LidarHelper* lidar_helper = nullptr;
    SharedData* sharedData = nullptr;
    LaserScanerControls* laser_controls = nullptr;
    QMutex pointsMaskMutex;

    void run();

    void is_start_detect(bool flag);
    void closed();
    std::map<std::string, LidarData> getLaserDetectResults();
    LidarHandler(LidarHelper* lidarHelper, LaserScanerControls* laserControls, SharedData* sharedData);
    ~LidarHandler();


};

#endif PDROBOT_LIDARHANDLER_H
