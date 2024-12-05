/*****************************************************************
 * \file   GlobalParameter.hpp
 * \brief
 *
 * \author anony
 * \date   October 2024
 *********************************************************************/
#pragma once

#include <map>
#include <QVector>

namespace GP
{
	enum class WorkingScenario
	{
		Top = 1,
		Cant,
		Side
	};

	enum class PositionType
	{
		Prepare = 1,
		Lift,
		Quit
	};

	struct PositionData
	{
		std::string brief;
		std::vector<double> value;
	};

	using PositionMap = std::map<std::pair<GP::WorkingScenario, GP::PositionType>, GP::PositionData>;

	extern WorkingScenario Working_Scenario;
	extern PositionMap Position_Map;

	static const int DOF = 10;

	extern double velLine;
	extern double velRotate;

	extern std::size_t CYLINDER_INDEX;
	extern std::size_t STEER_LEFT_INDEX;
	extern std::size_t STEER_RIGHT_INDEX;
	extern std::size_t WHEEL_LEFT_INDEX;
	extern std::size_t WHEEL_RIGHT_INDEX;
	extern std::size_t TOOL_LIFTING;

	extern double Lift_Distance_In_Parallel;
	extern double Max_Deviation_In_Parallel;
	extern double Min_Deviation_In_Parallel;

	extern double Distance_work;

	extern double Lift_Distance_In_FitBoard;
	extern double Max_Deviation_In_FitBoard;
	extern double Min_Deviation_In_FitBoard;

	extern double Line_Deviation_Threshold;

	extern std::vector<double> End_Vel_Limit;
	extern std::vector<double> End_Vel_Position;

	extern std::string Robot_IP;
	extern std::size_t Robot_Port;

	extern std::string IOA_IP;
	extern std::size_t IOA_Port;

	extern std::string IOB_IP;
	extern std::size_t IOB_Port;

	extern std::size_t Do_Weld_Parallel_Condition;

	extern double Laser_Valid_Threshold;
}