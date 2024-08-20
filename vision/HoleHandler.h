
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
#include "HoleHelper.h"
#include "CameraControls.h"
#include "../vision/ShareData.h"


// 视觉检测结果
struct HoleVisionRes{
    bool status;
    float offset_x;
    float offset_y;
    cv::Mat img_drawed;
    std::string cam_name;
};



class HoleHandler: public QThread
{
//  Q_OBJECT
public:

    HoleRes hole_res;
    bool detect_control_flag = false;
    bool thread_control_flag = true;
    std::map<std::string, HoleVisionRes> results;
    HoleHelper* hole_helper= nullptr;
    CameraControls* cam_controls = nullptr;

    void run();
    QMutex data_lock;
    SharedData* sharedData=nullptr;

    void is_start_detect(bool flag);
    void closed();
    HoleHandler(HoleHelper* hole,CameraControls* cam, SharedData* shardData);
    ~HoleHandler();





};

