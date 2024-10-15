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

		GP::Home_Position = m_root["home_point"].as<std::vector<double>>();
		GP::Prepare_Position = m_root["prepare_point"].as<std::vector<double>>();
		for (int i = 0; i < GP::DOF; i++)
		{
			GP::Home_Position_QV[i] = GP::Home_Position.at(i);

			GP::Prepare_Position_QV[i] = GP::Prepare_Position.at(i);
		}

		GP::CYLINDER_INDEX = m_root["CYLINDER_INDEX"].as<std::size_t>();
		GP::STEER_LEFT_INDEX = m_root["STEER_LEFT_INDEX"].as<std::size_t>();
		GP::STEER_RIGHT_INDEX = m_root["STEER_RIGHT_INDEX"].as<std::size_t>();
		GP::WHEEL_LEFT_INDEX = m_root["WHEEL_LEFT_INDEX"].as<std::size_t>();
		GP::WHEEL_RIGHT_INDEX = m_root["WHEEL_RIGHT_INDEX"].as<std::size_t>();

		GP::velLine = m_root["velLine"].as<double>();
		GP::velRotate = m_root["velRotate"].as<double>() / 57.3;
	}
}