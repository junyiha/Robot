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
#include "GocatorControls.h"
#include "../vision/ShareData.h"





class LidarHandler: public QThread
{
//  Q_OBJECT
public:


    bool detect_control_flag = false;
    bool thread_control_flag = true;
    std::map<std::string, LidarData> results;
    LidarHelper* lidar_helper= nullptr;
    GocatorControls* gocator_controls = nullptr;

    void run();
    SharedData* sharedData = nullptr;
    void is_start_detect(bool flag);
    void closed();
    LidarHandler(LidarHelper* lidar,GocatorControls* gocator, SharedData* sharedData);
    ~LidarHandler();

};
