//
// Created by csh_i on 2024/4/28.
//
#ifndef PDROBOT_LASERSCANERCONTROLS_H
#define PDROBOT_LASERSCANERCONTROLS_H

#include"MyBestfitLaserScaner.h"
#include<string>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<QThread>
#include<spdlog/spdlog.h>



//struct LidarData{
//    float dist;
//    float angle;
//    float gap;
//    bool flag=0;//是否有效
//    cv::Mat show;
//    QString error;
//};

class LaserScanerControls:public QThread{
    Q_OBJECT
public:
    //
    std::string goc_name;
    //相关配置
    std::map<std::string ,std::string> ip_config;
     std::map<std::string ,bool> direct_config;


    //创建雷达对象
    std::map<std::string, MyBestfitLaserScaner*> scaners;
    std::map<std::string, cv::Mat> pointCloudMasks;
    std::shared_ptr<spdlog::logger> logger;


    LaserScanerControls();
    ~LaserScanerControls();
    void getDataAll();
    void run() override;

    void closeAllLaserScaner();

};
#endif PDROBOT_LASERSCANERCONTROLS_H

