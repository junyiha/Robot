#ifndef GVL_H
#define GVL_H
#include <string>
#include "robot/Parameter.h"
#include "GlobalParameter.hpp"

const double Distance_Lift = 90; //调平位置
const double Distance_work = 90; //位置


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


const double velLine = 5;           //界面点动限速：平移
const double velRotate = 0.5/57.3;  //界面点动限速：旋转

const double JOINT_VEL_MOVE = 3;      //单位degree

const double POSITION_RESOLUTION = 0.5;    //单位mm
const double ROTATE_RESOLUTION  = 0.05/57.3;      //单位°

const double END_VEL_LIMIT[6] = {5,5,5,0.05,0.05,0.05}; //关节末端运动限速
const double END_VEL_POSITION[6] = {1,1,1,0.05,0.05,0.05}; //关节末端运动限速

const unsigned int CYLINDER_INDEX = 5; //推杆关节轴号索引
const unsigned int STEER_LEFT_INDEX = 11; //左舵轮索引
const unsigned int STEER_RIGHT_INDEX = 13; //右舵轮索引
const unsigned int WHEEL_LEFT_INDEX = 12; //左行走轮索引
const unsigned int WHEEL_RIGHT_INDEX = 14; //右行走轮索引
//const unsigned int TOOL_INDEX = 4; //
//const unsigned int PITCH_JOINT_INDEX = 4;//俯仰轴运动学模型索引

//
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
#endif // GVL_H