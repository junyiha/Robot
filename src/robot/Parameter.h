#pragma once
#pragma pack(push)
#pragma pack(8)
#include <stdio.h> //修改
#include <string.h>
#include <math.h>
#include <QVector>

#include "robot/NewParameter.hpp"

/* Parameter for Robot
 */
const long MAX_FREEDOM_LINK = 20;     // Link 最大自由度
const long MAX_FREEDOM_ROBOT = 30;    // robot最大自由度
const long LOST_COMM_THRESHOLD = 200; // 通讯超时周期数
const double MIN_VEL_LIMT = 0.001;    // 单位 °/s
const double MINIUM = exp(-10);
const double MAX = 10000000000;
const double MIN = 10000000000;

//------------------------------------------------------------------------------//
// LINK自由度配置
// link0 robot--10个电机    link1 地盘--4个电机
//------------------------------------------------------------------------------//
const long LINK_FREEDOM[6] = { 11, 4, 0, 0, 0, 0 };

//------------------------------------------------------------------------------//
// 轴限位
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 250,  // 0 升降
                                                         110,  // 1 前后
                                                         110,  // 2 左右
                                                         95,   // 3 腰旋转
                                                         105,  // 4 腰俯仰
                                                         1100, // 5 臂俯仰
                                                         1700, // 6 筒伸缩
                                                         30,   // 7 筒旋转
                                                         90,   // 8 腕俯仰
                                                         1490, // 9 末端升降
                                                         620,  // 10 末端旋转
                                                         MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_0_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 0,    // 0 升降
                                                         -110, // 1 前后
                                                         -110, // 2 左右
                                                         -95,  // 3 腰旋转
                                                         75,   // 4 腰俯仰
                                                         790,  // 5 臂俯仰
                                                         1070, // 6 筒伸缩
                                                         -30,  // 7 筒旋转
                                                         -100, // 8 腕俯仰
                                                         855, // 9 末端升降
                                                         520,  // 10 末端旋转
                                                         -MIN, -MIN, -MIN, -MIN, -MIN, -MIN, -MIN, -MIN, -MIN };

//------------------------------------------------------------------------------//
// 最大速度
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 7, 20, 20, 10, 3, 30, 27, 5, 10, 4, 3, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };

#pragma pack(pop)
