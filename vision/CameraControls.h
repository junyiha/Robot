#ifndef CAMERACONTROL_H
#define CAMERACONTROL_H
#include<string>
#include<iostream>
#include"Camera.h"
#include"FFmpegCamera.h"
#include <unordered_map>
#include<QMutex>

#define NUM_EDGE 5;
#define NUM_HOLE 4;

struct CameraRes{
    cv::Mat img_data; //相机数据
    bool status;  //相机状态
    std::string cam_name;  //相机名称
};


//相机控制类 ，管理多个相机，并获取相机数据
class CameraControls{

public:

    std::string type; //相机类型
    std::map<std::string, FFmpegCamera*> cameras;

    std::map<std::string, CameraRes> cameras_data;  //相机数据存储
    std::map<std::string, unsigned> rotate_types;


    //方法函数
    CameraControls(std::string type);
    ~CameraControls();

    QMutex data_lock;
    //获取多个相机的数据
    void get_camera_data();

    //获取指定名称相机的数据
    CameraRes get_camera_data_by_name(std::string cam_name);

    //关闭多个相机
    void close_cameras();
    //关闭指定名称的相机
    void close_cameras_by_name(std::string cam_name);
};


#endif // CAMERACONTROL_H
