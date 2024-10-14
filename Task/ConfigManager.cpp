#include "ConfigManager.hpp"

namespace Config
{
	ConfigManager::ConfigManager()
	{
		m_root = YAML::LoadFile(m_path);
		try
		{
			ParseConfiguration();
		}
		catch (...)
		{
			std::cerr << "parse config.yaml failed, please check the config.yaml file!!!\n";
			abort();
		}
	}

	ConfigManager::~ConfigManager()
	{

	}

	void ConfigManager::ParseConfiguration()
	{
		GP::End_Vel_Limit = m_root["end_vel_limit"].as<std::vector<double>>();
		GP::End_Vel_Position = m_root["end_vel_position"].as<std::vector<double>>();

		GP::Robot_IP = m_root["robot_ip"].as<std::string>();
		GP::Robot_Port = m_root["robot_port"].as<std::size_t>();

		GP::IOA_IP = m_root["IOA_ip"].as<std::string>();
		GP::IOA_Port = m_root["IOA_port"].as<std::size_t>();

		auto home_point = m_root["home_point"].as<std::vector<double>>();
		auto prepare_point = m_root["prepare_point"].as<std::vector<double>>();
		for (int i = 0; i < GP::DOF; i++)
		{
			GP::Home_Position[i] = home_point.at(i);
			GP::Home_Position_QV[i] = home_point.at(i);

			GP::Prepare_Position[i] = prepare_point.at(i);
			GP::Prepare_Position_QV[i] = prepare_point.at(i);
		}
	}
}