#pragma once
#pragma pack(push)
#pragma pack(8) 
#include <stdio.h>//修改
#include <string.h>
#include <math.h>

/* Parameter for Robot 
*/

const long MAX_FREEDOM_LINK = 20;			//Link 最大自由度
const long MAX_FREEDOM_ROBOT = 30;			//robot最大自由度
const long LOST_COMM_THRESHOLD = 200;		//通讯超时周期数
const double MIN_VEL_LIMT = 0.001;			//单位 °/s
const double MINIUM = exp(-10);
const double MAX = 10000000000;
const double MIN = 10000000000;
//------------------------------------------------------------------------------//
//LINK自由度配置
//link0 robot--10个电机    link1 地盘--4个电机
//------------------------------------------------------------------------------//
const long LINK_FREEDOM[6] = { 10, 6, 0, 0, 0, 0};

//------------------------------------------------------------------------------//
//轴限位
//------------------------------------------------------------------------------//
                                                        //升降      前后      左右        腰旋转    腰俯仰      臂俯仰        筒伸缩       筒旋转       腕俯仰     末端升降
                                                        // 0        1         2           3        4           5           6           7           8           9
const double LINK_0_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 300, 100,  145 , 170,  15,   60,     1550,       30 ,       90,           1100, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_0_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 0  , -100, -145, -170 ,-15,  0,      1200 ,       -30,       -90,             800,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };

//const double LINK_0_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = {  64,  144,  257,184, 14, 1147, 480, 150, 99,  250, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
//const double LINK_0_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -64, -144,   3, -4, -14, 834,   4, -150, -47,   3,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
const double LINK_1_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { MAX, 95, MAX, 95, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_1_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -MIN,-95,-MIN,-95,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
const double LINK_2_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { MAX,  MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_2_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
const double LINK_3_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_3_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
const double LINK_4_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_4_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
const double LINK_5_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_5_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };

//------------------------------------------------------------------------------//
//最大速度
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 10, 10, 10, 5, 2, 2, 10, 0, 0, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
//const double LINK_0_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 40, 40, 10, 4, 10, 24, 5, 2, 2, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_1_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 2, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_2_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 100, 30, 100, 30, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_3_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_4_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_5_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };

//------------------------------------------------------------------------------//
//减速比
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_RATIO[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const double LINK_1_JOINT_RATIO[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const double LINK_2_JOINT_RATIO[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const double LINK_3_JOINT_RATIO[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const double LINK_4_JOINT_RATIO[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const double LINK_5_JOINT_RATIO[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };

//------------------------------------------------------------------------------//
//电机方向
//------------------------------------------------------------------------------//
const long LINK_0_JOINT_DIRECTION[MAX_FREEDOM_LINK] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const long LINK_1_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const long LINK_2_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const long LINK_3_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const long LINK_4_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
const long LINK_5_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };

//------------------------------------------------------------------------------//
//编码器偏差
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
const double LINK_1_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
const double LINK_2_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
const double LINK_3_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
const double LINK_4_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
const double LINK_5_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

#pragma pack(pop)
