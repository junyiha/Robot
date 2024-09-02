#ifndef CROBOTKINECTMODEL_H
#define CROBOTKINECTMODEL_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <cmath>
#include <Windows.h>
#include <QDebug>
#include <qmath.h>
#include "Transformation.h"
#include "../GVL.h"
#include <spdlog/spdlog.h>

class CRobotKinectModel
{
public:
    /**
    * @brief 机器人运动模型
    * @param  Input_DH_List 关节DH参数队列
    * @param  Tool_DH       工具转换矩阵
    */
    CRobotKinectModel(std::vector<CTransformation> Input_DH_List, CTransformation Tool_DH = CTransformation(0, 0, 0, 0, Revolute));
    ~CRobotKinectModel(){


  //      m_SerialLink.swap(vector<CTransformation>,allocator<CTransformation>);
 //       std::vector<int>().swap(Valid_Index);
    };

    /**
    * @brief 更新关节位置
    * @param  Joint_Value 关节位置
    */
    void setJointPos(std::vector<std::pair<double, bool>> Joint_Value);

    /**
    * @brief 设定工具位置
    * @param  len       末端工具长度
    * @param  rot       工具旋转量，绕工具z轴
    */
    void setToolPos(double len, double rot);

    /**
    * @brief 根据末端速度求解关节速度：考虑限位/不考虑限位
    * @param  Tar_Speed       末端目标速度
    * @param  Ref_End         移动参考，true：末端坐标系，false:基坐标系
    * @return 关节速度向量
    */
    Eigen::VectorXd getJointVel(Eigen::VectorXd Tar_Speed,bool Ref_End=false);
    Eigen::VectorXd getJointVel_nolimit(Eigen::VectorXd Tar_Speed,bool Ref_End=false);


    /**
    * @brief 获取机器人末端速度
    * @return  基坐标下机器人末端位置
    */
    Eigen::VectorXd getEndPosition();


    /**
    * @brief 根据目标位置计算末端移动矢量(基坐标)
    * @param  tarEndPos 目标位姿矩阵
    * @return  基坐标下速度矢量
    */
    Eigen::VectorXd getTarSpeed(Eigen::Matrix4d tarEndPos);

    /**
    * @brief 根据目标位置计算末端移动矢量(基坐标)
    * @param  tarEndPos 目标位姿向量
    * @return  基坐标下速度矢量
    */
    Eigen::VectorXd getTarSpeed(Eigen::VectorXd tarEndPos);


    Eigen::MatrixXd getTarEndPos(Eigen::VectorXd &deltaPos);

    /**
    * @brief 获取末端位置
    * @return  返回末端位姿矩阵
    */
    Eigen::Matrix4d getEndPose(){return TransMatLisForward.back();}

private:
    /**
    * @brief 将坐标从工具坐标系换算到基坐标系
    * @param  工具坐标下机器人目标位姿
    * @return 基础坐标下机器人目标位姿
    */
    Eigen::VectorXd end2base(Eigen::VectorXd end);

    /**
    * @brief 计算机器人转换矩阵，更新TransMatLisForward,TransMatLisBackward
    * @
    */
    void Transformation();

    /**
    * @brief FABRIK正逆迭代法计算雅克比矩阵
    * @param
    */
    void calJacobian();

    /**
    * @brief 推缸行程转为关节角度
    * @param D2 推杆长度
    */
    double Joint2Fkine(double D2);

    /**
    * @brief 推缸行程转为关节角度
    * @param D2 推杆长度
    * @param Theta2Dot 转动角度
    */
    double Joint2InvKine(double Theta2Dot,double D2);



    std::vector<CTransformation> m_SerialLink;
    CTransformation         m_ToolLink = CTransformation(0, 0, 0, 0, Revolute);

    std::deque<Eigen::Matrix4d>  TransMatLisForward;		//正向变换矩阵  即为各个节点的位姿  [0  1 2 3 4 5 6 ……]
    std::deque<Eigen::Matrix4d>  TransMatLisBackward;	//逆向变换矩阵  为雅克比做准备
    Eigen::MatrixXd         JacobianMat;			//雅克比矩阵
    Eigen::MatrixXd         JacobianMatInv;         //雅克比逆矩阵(广义逆)
    std::vector<int>        Valid_Index;            //未锁定轴索引
    double                  m_CylinderLen;

    std::shared_ptr<spdlog::logger> log;



    const double            Partical_Delta = 0.0001; //雅克比步长 不建议改变

};

#endif // CROBOTKINECTMODEL_H
