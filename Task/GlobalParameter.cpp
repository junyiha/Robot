#include "GlobalParameter.hpp"

namespace GP
{
	WorkingScenario Working_Scenario{WorkingScenario::Top};
	PositionMap Position_Map{};

	double velLine{0.0};	
	double velRotate{0.0};	

	std::size_t CYLINDER_INDEX{0};
	std::size_t STEER_LEFT_INDEX{0};
	std::size_t STEER_RIGHT_INDEX{0};
	std::size_t WHEEL_LEFT_INDEX{0};
	std::size_t WHEEL_RIGHT_INDEX{0};
	std::size_t TOOL_LIFTING{ 0 };

	double Lift_Distance_In_Parallel{0.0};  
	double Max_Deviation_In_Parallel{0.0};  
	double Min_Deviation_In_Parallel{0.0};   

	double Distance_work{0.0}; 

	double Lift_Distance_In_FitBoard{0.0};    
	double Max_Deviation_In_FitBoard{0.0};   
	double Min_Deviation_In_FitBoard{0.0};    

	double Line_Deviation_Threshold{ 0.0 };

	std::vector<double> End_Vel_Limit(6, 0.0);
	std::vector<double> End_Vel_Position(6, 0.0);

	std::string Robot_IP;
	std::size_t Robot_Port;

	std::string IOA_IP;
	std::size_t IOA_Port;

	std::string IOB_IP;
	std::size_t IOB_Port;

	std::size_t Do_Weld_Parallel_Condition;

	double Laser_Valid_Threshold;
}