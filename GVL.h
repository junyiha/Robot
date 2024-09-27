#ifndef GVL_H
#define GVL_H
#include "robot/Parameter.h"

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

const double JOINT_VEL_LIMIT[20] = {2,2,2,2,1,10,10,1,2,5,5,0,0,0,0,0,0,0,0,0};

const unsigned int CYLINDER_INDEX = 5; //推杆关节轴号索引
const unsigned int STEER_LEFT_INDEX = 11; //左舵轮索引
const unsigned int STEER_RIGHT_INDEX = 13; //右舵轮索引
const unsigned int WHEEL_LEFT_INDEX = 12; //左行走轮索引
const unsigned int WHEEL_RIGHT_INDEX = 14; //右行走轮索引
//const unsigned int TOOL_INDEX = 4; //
const unsigned int PITCH_JOINT_INDEX = 4;//俯仰轴运动学模型索引


//取板位置
const double LOAD_POSTION[20] ={10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,0,0};
//安装举升位置
const double INTALL_POSTION[20] ={10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,0,0};
//TarPositionA
const double TarPositionA[20] ={10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,0,0};

//A1 末端速度限制
const double A1_VEL[6]={1,1,1,0.2,0.2,0.2};

// 放钉位置
const double Postion_Home[10]    =    {16.020,-90,0,10.23,76,801, 1095, 0, -12, 1109};						
const QVector<double> Postion_Home_qv = {16.020,-90,0,10.23,76,801, 1095, 0, -12, 1109};					

//碰钉准备位置
const double Postion_Prepare[10]    = {16.02,-30,0,10.23,76.01,1034.45,1177.37, 0.8, -71.17, 1162.56};		
const QVector<double> Postion_Prepare_qv = {16.02,-30,0,10.23,76.01,1034.45,1177.37, 0.8, -71.17, 1162.56}; 

//退出位置
const double Postion_Quit[10]    =    {16.02,-30,0,10.23,76.01,900,1177.37, 0.8, -71.17, 1100.56};			
const QVector<double> Postion_Quit_qv    =    {16.02,-30,0,10.23,76.01,900,1177.37, 0.8, -71.17, 1100.56};	

// 装板机器人: 装板位
const double ZB_Position_Home[11] = { 2.21, -0.25, 1.84, 10, 76.41, 817.17, 1142.71, 1.86, -4.23, 901.36 , 567};
const QVector<double> ZB_Position_Home_qv = {2.21, -0.25, 1.84, 10, 76.41, 817.17, 1142.71, 1.86, -4.23, 901.36 , 567};

// 装板机器人: 举升位
const double ZB_Position_Prepare[11] = { 97.45, 46.27, 11.63, 9.75, 78.81, 1043.36, 1194.38, 0.55, -62.84, 885.0 , 567};
const QVector<double> ZB_Position_Prepare_qv = { 97.45, 46.27, 11.63, 9.75, 78.81, 1043.36, 1194.38, 0.55, -62.84, 885.0 , 567};

//robotcomm的ip和端口"192.168.1.130" 5999
const char g_str_robotip[] = "192.168.1.130";
//const int g_i_robotport = 5999;  old
const int g_i_robotport = 6999;

//IOAcomm的ip和端口"192.168.1.201" 5999
const char g_str_IOAip[] = "192.168.1.201";
const int g_i_IOAport = 5999;

//IOAcomm的ip和端口"192.168.1.202" 5999
const char g_str_IOBip[] = "192.168.1.202";
const int g_i_IOBport = 5999;

//-------临时
#define LINK0 0
#define LINK1 1
#define LINK2 2
#define JOINTNUM 10
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

enum eDistanceIndex
{
    d1 = 0,
    d2,
    d3,
    d4
};



//********************************临时加
//传感器个数
#define SUM_Lidar 5
#define SUM_Edge  10
#define SUM_Hole  4
#define SUM_Laser 4 //点激光
//报告中数据个数
#define SUM_RepLidar 5
#define SUM_RepEdge  10 //导出报告中需要10组边线距
//图像显示个数
#define SUM_ShwEdge 10
#define SUM_ShwHole 4
#define SUM_ShwLidar 5


#endif // GVL_H