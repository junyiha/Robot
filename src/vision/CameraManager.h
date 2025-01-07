//
// Created by csh_i on 2024/6/25.
//

#ifndef HK_DEMO2_CAMERAMANAGER_H
#define HK_DEMO2_CAMERAMANAGER_H
#include<QDebug>
#include <QMutex>
#include <QThread>

#include <map>
#include <iostream>
#include <vector>

#include <spdlog/spdlog.h>
#include "MvCamera.h"

#include"HKCameraControls.h"
#include "utils.h"


/*****************************Multi Camera Manager  *****************************************/
class CameraManager :public QThread {
public:

    std::map<std::string, std::string> camera_info;
    std::map<std::string, HKCameraControls*> cameraList;
    std::map<std::string, cv::Mat> cameraImages;
    std::map<std::string, cv::Mat> cameraImagesResize;
    std::map<std::string, std::string> serial2names;
    std::map<std::string, MV_CC_DEVICE_INFO> cameraInfoMap;
    std::map<std::string, int> imageCorrectMap;
    int imgUiH = 280;
    int imgUiW = 400;


    MV_CC_DEVICE_INFO_LIST  m_stDevList = { 0 };



    const char* ethIp = "192.168.1.10";
    std::shared_ptr<spdlog::logger> logger;

    CameraManager();
    ~CameraManager();
    QMutex dataMutex;
    QMutex conMutex;
    std::vector<bool> cameraConnectStatus;
    void run() override;

    std::vector<bool>getCameraConnectStatus();



    bool openCamera(std::string camera_name);  // 打开相机
    bool closeCamera(std::string camera_name);//  关闭相机
    void openCameraAll();  //   打开所有相机
    bool closeCameraAll(); //   关闭所有相机
    cv::Mat getImage(std::string camera_name); // 获取单个相机图像
    void getImageAll(std::string camType = "all"); // 获取所有相机图像
    void getImageAllByResizeAndCorrect(std::string camType = "all"); // 获取所有相机图像

    void getDeviceList(); // 获取设备列表
    std::vector<bool> checkCameraIsAccessible(); // 检查相机是否可以访问
    std::vector<bool> getCameraOpenedInfo(); // 检查相机是否打开
    bool camerasIsOpened();


    std::map<std::string, cv::Mat> getCameraImages(std::string camType = "all"); // 获取所有相机图像
    std::map<std::string, cv::Mat> CameraManager::getCameraImagesResized(std::string camType = "all");
    void getImageAllNonResize(std::string camType = "all");

    void closeAllCameraThread();

private:

    MV_CC_DEVICE_INFO       m_stDevInfo = { 0 };



};
#endif //HK_DEMO2_CAMERAMANAGER_H
