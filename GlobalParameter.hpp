/*****************************************************************//**
 * \file   GlobalParameter.hpp
 * \brief  
 * 
 * \author anony
 * \date   October 2024
 *********************************************************************/
#pragma once 

#include <QVector>

namespace GP
{
	static const int DOF = 11;

	extern double velLine;		//界面点动限速：平移
	extern double velRotate;	//界面点动限速：旋转

	extern std::size_t CYLINDER_INDEX;		// 推杆关节轴号索引
	extern std::size_t STEER_LEFT_INDEX;	// 左舵轮索引
	extern std::size_t STEER_RIGHT_INDEX;	// 右舵轮索引
	extern std::size_t WHEEL_LEFT_INDEX;	// 左行走轮索引
	extern std::size_t WHEEL_RIGHT_INDEX;	// 右行走轮索引
	extern std::size_t TOOL_LIFTING;		// 工具升降索引

	// 机器人: 准备位
	extern std::vector<double> Home_Position;
	extern QVector<double> Home_Position_QV;

	// 机器人: 举升位
	extern std::vector<double> Prepare_Position;
	extern QVector<double> Prepare_Position_QV;

	// 关节末端运动限速
	extern std::vector<double> End_Vel_Limit;
	extern std::vector<double> End_Vel_Position;

	extern std::string Robot_IP;
	extern std::size_t Robot_Port;

	extern std::string IOA_IP;
	extern std::size_t IOA_Port;
}