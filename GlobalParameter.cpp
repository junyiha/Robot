#include "GlobalParameter.hpp"

namespace GP
{
	double Home_Position[DOF];
	QVector<double> Home_Position_QV = QVector<double>(DOF, 0.0);

	double Prepare_Position[DOF];
	QVector<double> Prepare_Position_QV(DOF, 0.0);

	std::vector<double> End_Vel_Limit(6, 0.0);
	std::vector<double> End_Vel_Position(6, 0.0);

	std::string Robot_IP;
	std::size_t Robot_Port;

	std::string IO_IP;
	std::size_t IO_Port;
}