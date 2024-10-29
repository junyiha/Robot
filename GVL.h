﻿#ifndef GVL_H
#define GVL_H
#include "robot/Parameter.h"
#include "GlobalParameter.hpp"

const double DistanceB = 80;//2024年5月17日20点20分 修改，原始为80
const double DistanceC = 30;
const double DistanceD = 0;

const double x_Laser = 510; //虚拟点位置
const double y_Laser = 1515;    //虚拟点位置
const double x_ProfileLaser = 495; //虚拟点位置
const double y_ProfileLaser = 1300;//虚拟点位置
const double x_hole = 440;
const double y_hole = 1460;
const double x_camera = 510;
const double y_camera = 1075;


const double JOINT_VEL_MOVE = 3;      //单位degree

const double POSITION_RESOLUTION = 0.5;    //单位mm
const double ROTATE_RESOLUTION  = 0.05/57.3;      //单位°

//                                   0底升 1前后  2左右  3腰转 4大臂  5腕部  6末端
const double JOINT_VEL_LIMIT[20] = {2,2,2,2,1,10,10,1,2,5,0,0,0,0,0,0,0,0,0,0};


//-------临时
#define DOBYTENUM 6

enum eJointIndex
{
    Joint1 = 0,
    Joint2,
    Joint3,
    Joint4,
    Joint5,
    Joint6,
    Joint7,
    JointNum = 7,
};

#define SUM_ShwHole 4

#endif // GVL_H
