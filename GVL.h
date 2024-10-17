#ifndef GVL_H
#define GVL_H
#include <string>
#include "robot/Parameter.h"
#include "GlobalParameter.hpp"

const double Lift_Distance_In_Parallel = 90.0;  // 调平状态下举升的位置
const double Max_Deviation_In_Parallel = 50.0;  // 调平允许最大偏差
const double Min_Deviation_In_Parallel = 5.0;   // 调平允许最小偏差

const double Distance_work = 90; //位置

const double Lift_Distance_In_FitBoard = 6.0;    // 贴合状态下举升的位置
const double Max_Deviation_In_FitBoard = 20.0;   // 贴合允许偏差
const double Min_Deviation_In_FitBoard = 6.0;    // 贴合允许偏差

const double LINE_DEVIATION_THRESHOLD = 2.0;    //边线调整允许偏差  1.0

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