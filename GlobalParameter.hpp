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

	// 机器人: 准备位
	extern double Home_Position[];
	extern QVector<double> Home_Position_QV;

	// 机器人: 举升位
	extern double Prepare_Position[];
	extern QVector<double> Prepare_Position_QV;

	// 关节末端运动限速
	extern std::vector<double> End_Vel_Limit;
	extern std::vector<double> End_Vel_Position;

	extern std::string Robot_IP;
	extern std::size_t Robot_Port;

	extern std::string IOA_IP;
	extern std::size_t IOA_Port;
}