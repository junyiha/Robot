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

	static const int DOF = 11;

	extern double velLine;	 // 界面点动限速：平移
	extern double velRotate; // 界面点动限速：旋转

	extern uint CYLINDER_INDEX;	  // 推杆关节轴号索引
	extern uint STEER_LEFT_INDEX;  // 左舵轮索引
	extern uint STEER_RIGHT_INDEX; // 右舵轮索引
	extern uint WHEEL_LEFT_INDEX;  // 左行走轮索引
	extern uint WHEEL_RIGHT_INDEX; // 右行走轮索引
	extern uint TOOL_LIFTING;	  // 工具升降索引

	extern double Lift_Distance_In_Parallel; // 调平状态下举升的位置
	extern double Max_Deviation_In_Parallel; // 调平允许最大偏差
	extern double Min_Deviation_In_Parallel; // 调平允许最小偏差

	extern double Distance_work; // 位置

	extern double Lift_Distance_In_FitBoard; // 贴合状态下举升的位置
	extern double Max_Deviation_In_FitBoard; // 贴合允许偏差
	extern double Min_Deviation_In_FitBoard; // 贴合允许偏差

	extern double Line_Deviation_Threshold; // 边线调整允许偏差

	extern double Second_Push_Distance; // 第二次举升距离
	extern double Second_Quit_Distance; // 第二次退出距离

	extern std::map<int, double> Vision_Replace_Offset_Map; // 视觉替换偏移量

	// 关节末端运动限速
	extern std::vector<double> End_Vel_Limit;

	extern std::string Robot_IP;
	extern std::size_t Robot_Port;

	extern std::string IOA_IP;
	extern std::size_t IOA_Port;
}