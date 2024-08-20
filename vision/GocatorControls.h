//
// Created by csh_i on 2024/4/28.
//

#ifndef CLION_TEST_GOCATORCONTROLS_H
#define CLION_TEST_GOCATORCONTROLS_H
#include"Gocator.h"
#include<string>
#include<iostream>
#include<opencv2/opencv.hpp>
#include"LidarHelper.h"


//struct LidarData{
//    float dist;
//    float angle;
//    float gap;
//    bool flag=0;//是否有效
//    cv::Mat show;
//    QString error;
//};

class GocatorControls {
public:
    //
    std::string goc_name;
    //相关配置
    std::map<std::string ,std::string> ip_config;

    //创建雷达对象
    std::map<std::string, Gocator*>  gocators;
    std::map<std::string, cv::Mat> lidars;



    GocatorControls();
    ~GocatorControls();
    void get_data();

};

#endif //CLION_TEST_GOCATORCONTROLS_H
