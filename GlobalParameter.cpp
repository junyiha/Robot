#include "GlobalParameter.hpp"

namespace GP
{
	double velLine{0.0};	
	double velRotate{0.0};	

	std::size_t CYLINDER_INDEX{0};
	std::size_t STEER_LEFT_INDEX{0};
	std::size_t STEER_RIGHT_INDEX{0};
	std::size_t WHEEL_LEFT_INDEX{0};
	std::size_t WHEEL_RIGHT_INDEX{0};

	std::vector<double> Home_Position(DOF, 0.0);
	QVector<double> Home_Position_QV = QVector<double>(DOF, 0.0);

	std::vector<double> Prepare_Position(DOF, 0.0);
	QVector<double> Prepare_Position_QV(DOF, 0.0);

	std::vector<double> End_Vel_Limit(6, 0.0);
	std::vector<double> End_Vel_Position(6, 0.0);

	std::string Robot_IP;
	std::size_t Robot_Port;

	std::string IOA_IP;
	std::size_t IOA_Port;
}