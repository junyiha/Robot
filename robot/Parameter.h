#pragma once
#pragma pack(push)
#pragma pack(8) 
#include <stdio.h>//修改
#include <string.h>
#include <math.h>

/* Parameter for Robot 
*/

//link1 9个电机    link2 1个腰俯仰电机   link3 4个轮子电机
#define LINK_FREEDOM1_NUM   7
#define LINK_FREEDOM2_NUM   4
#define LINK_FREEDOM3_NUM   0
const long MAX_FREEDOM_LINK = 20;			//Link 最大自由度
const long MAX_FREEDOM_ROBOT = 30;			//robot最大自由度
const long LOST_COMM_THRESHOLD = 200;		//通讯超时周期数
const double MIN_VEL_LIMT = 0.001;			//单位 °/s
const double MINIUM = exp(-10);
const double MAX = 10000000000;
const double MIN = 10000000000;
//------------------------------------------------------------------------------//
//LINK自由度配置
//------------------------------------------------------------------------------//
const long LINK_FREEDOM[6] = { LINK_FREEDOM1_NUM, LINK_FREEDOM2_NUM, LINK_FREEDOM3_NUM, 0, 0, 0};

//------------------------------------------------------------------------------//
//轴限位
//------------------------------------------------------------------------------//
                                                        //上下        前后      左右          腰旋转       摆动        俯仰        筒伸缩       筒旋转       末端俯仰     末端升降
                                                        // 0        1            2           3          4           5           6           7           8           9
const double LINK_0_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = {  1000,       62,      145 ,      170,        10,        45,         1550,       0 ,       0,           0, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_0_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 5  ,        -62,     -145,       -170 ,     -10,      -45,         1200 ,       0,       0,             0,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
/*                     下位机限位                                                                         17          1150        480
 *                                                                                                      -17         840         0
 */
//const double LINK_0_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = {  64,  144,  257,184, 14, 1147, 480, 150, 99,  250, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
//const double LINK_0_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -64, -144,   3, -4, -14, 834,   4, -150, -47,   3,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
const double LINK_1_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 15, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_1_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -15,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
const double LINK_2_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { MAX,  90, MAX, 90, MAX, MAX, MAX, MAX, MAX, MAX,MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX };
const double LINK_2_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -MIN,-90,-MIN,-90,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN };
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
