#ifndef GETPIC_H
#define GETPIC_H
#include <string>
#include <iostream>
#include <time.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <QDebug>
#include <QLabel>
#include <QObject>
#include <QThread>
#include <QApplication>
#include<QMutex>
#include<queue>




class Camera: public QThread
{
//  Q_OBJECT
public:
    //相机相关参数
    std::string rtsp;
    std::string camera_name; //相机名称
    cv::Mat img;
    cv::VideoCapture cap; //相机

    //进程运行状态参数
    bool isOpened=false; //相机是否打开
    bool running=false;
    bool thread_control_flag= true;
    std::queue<cv::Mat> myQueue;
    unsigned max_size = 5;

    QMutex img_lock;

    unsigned rotate=0;  //这里改了无效，得从getvisiondata.h的VisionSensor中改 1顺时针90 ，2顺180 3逆时针90  0表示不发生任何旋转
    Camera(std::string cam_name, std::string rstp);
    ~Camera();

    //相机控制函数
    void run();
    void open_camera();
    void stop();

};
#endif // GETPIC_H
