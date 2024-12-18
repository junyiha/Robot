#include "GlobalParameter.hpp"

namespace GP
{
	WorkingScenario Working_Scenario{ WorkingScenario::Top };
	PositionMap Position_Map{};

	double velLine{ 0.0 };
	double velRotate{ 0.0 };

	uint CYLINDER_INDEX{ 0 };
	uint STEER_LEFT_INDEX{ 0 };
	uint STEER_RIGHT_INDEX{ 0 };
	uint WHEEL_LEFT_INDEX{ 0 };
	uint WHEEL_RIGHT_INDEX{ 0 };
	uint TOOL_LIFTING{ 0 };

	double Lift_Distance_In_Parallel{ 0.0 };
	double Max_Deviation_In_Parallel{ 0.0 };
	double Min_Deviation_In_Parallel{ 0.0 };

	double Distance_work{ 0.0 };

	double Lift_Distance_In_FitBoard{ 0.0 };
	double Max_Deviation_In_FitBoard{ 0.0 };
	double Min_Deviation_In_FitBoard{ 0.0 };

	double Line_Deviation_Threshold{ 0.0 };

	std::vector<double> Lift_Position(DOF, 0.0);

	std::vector<double> Prepare_Position(DOF, 0.0);

	std::vector<double> End_Vel_Limit(6, 0.0);
	std::vector<double> End_Vel_Position(6, 0.0);

	std::string Robot_IP;
	std::size_t Robot_Port;

	std::string IOA_IP;
	std::size_t IOA_Port;
}