#ifndef GOCATOR_H
#define GOCATOR_H

#include<QImage>
#include <QLabel>

#include <stdio.h>
#include <cstdio>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Windows.h>
#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <GoSdk/GoSdk.h>
#include <stdlib.h>
#include <memory.h>


#define RECEIVE_TIMEOUT         (20000000)
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data.
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.
//#define SENSOR_IP               "192.168.1.10"
#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

typedef struct ProfilePoint
{
    double x;   // x-coordinate in engineering units (mm) - position along laser line
    double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
    unsigned char intensity;
} ProfilePoint;


class Gocator
{
public:
//    QLabel* label;
    std::string ip="192.168.1.10";
    const char * SENSOR_IP = ip.c_str();
    cv::Mat lidar_data;

    std::string goc_name; //雷达名称
    Gocator();
    int getData(); //获取雷达数据
    bool isleft;
    bool revAngle;
};
#endif // GOCATOR_H
