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

	extern double Home_Position[];
	extern QVector<double> Home_Position_QV;

	extern double Prepare_Position[];
	extern QVector<double> Prepare_Position_QV;

	extern std::vector<double> End_Vel_Limit;
	extern std::vector<double> End_Vel_Position;

	extern std::string Robot_IP;
	extern std::size_t Robot_Port;

	extern std::string IO_IP;
	extern std::size_t IO_Port;
}