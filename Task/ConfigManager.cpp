#include "ConfigManager.hpp"

namespace Config
{
	ConfigManager::ConfigManager()
	{
		log = spdlog::get("logger");

		if (!LoadConfiguration())
		{
			throw std::bad_exception();
		}
	}

	ConfigManager::~ConfigManager()
	{

	}

	bool ConfigManager::LoadConfiguration()
	{
		bool res{ true };
		try
		{
			m_root = YAML::LoadFile(m_path);
			ParseConfiguration();
		}
		catch (YAML::BadFile)
		{
			log->error("Invalid configuration file path: {}", m_path);
			res = false;
		}
		catch (...)
		{
			log->error("parse config.yaml failed, please check the config.yaml file!!!");
			res = false;
		}

		return res;
	}

	void ConfigManager::ParseConfiguration()
	{
		GP::End_Vel_Limit = m_root["end_vel_limit"]["value"].as<std::vector<double>>();
		GP::End_Vel_Position = m_root["end_vel_position"]["value"].as<std::vector<double>>();

		GP::Robot_IP = m_root["robot_ip"]["value"].as<std::string>();
		GP::Robot_Port = m_root["robot_port"]["value"].as<std::size_t>();

		GP::IOA_IP = m_root["IOA_ip"]["value"].as<std::string>();
		GP::IOA_Port = m_root["IOA_port"]["value"].as<std::size_t>();

		GP::IOB_IP = m_root["IOB_ip"]["value"].as<std::string>();
		GP::IOB_Port = m_root["IOB_port"]["value"].as<std::size_t>();

		GP::Home_Position = m_root["home_point"]["value"].as<std::vector<double>>();
		GP::Prepare_Position = m_root["prepare_point"]["value"].as<std::vector<double>>();
		for (int i = 0; i < GP::DOF; i++)
		{
			GP::Home_Position_QV[i] = GP::Home_Position.at(i);

			GP::Prepare_Position_QV[i] = GP::Prepare_Position.at(i);
		}

		GP::CYLINDER_INDEX = m_root["CYLINDER_INDEX"]["value"].as<std::size_t>();
		GP::STEER_LEFT_INDEX = m_root["STEER_LEFT_INDEX"]["value"].as<std::size_t>();
		GP::STEER_RIGHT_INDEX = m_root["STEER_RIGHT_INDEX"]["value"].as<std::size_t>();
		GP::WHEEL_LEFT_INDEX = m_root["WHEEL_LEFT_INDEX"]["value"].as<std::size_t>();
		GP::WHEEL_RIGHT_INDEX = m_root["WHEEL_RIGHT_INDEX"]["value"].as<std::size_t>();
		GP::TOOL_LIFTING = m_root["TOOL_LIFTING"]["value"].as<std::size_t>();

		GP::velLine = m_root["velLine"]["value"].as<double>();
		GP::velRotate = m_root["velRotate"]["value"].as<double>() / 57.3;

		GP::Lift_Distance_In_Parallel = m_root["Lift_Distance_In_Parallel"]["value"].as<double>();
		GP::Max_Deviation_In_Parallel = m_root["Max_Deviation_In_Parallel"]["value"].as<double>();
		GP::Min_Deviation_In_Parallel = m_root["Min_Deviation_In_Parallel"]["value"].as<double>();
		GP::Distance_work = m_root["Distance_work"]["value"].as<double>();
		GP::Lift_Distance_In_FitBoard = m_root["Lift_Distance_In_FitBoard"]["value"].as<double>();
		GP::Max_Deviation_In_FitBoard = m_root["Max_Deviation_In_FitBoard"]["value"].as<double>();
		GP::Min_Deviation_In_FitBoard = m_root["Min_Deviation_In_FitBoard"]["value"].as<double>();
		GP::Line_Deviation_Threshold = m_root["Line_Deviation_Threshold"]["value"].as<double>();
	}

	bool ConfigManager::WriteToFile()
	{
		bool res{ false };

		std::ofstream fout(m_path);
		if (fout)
		{
			fout << m_root;
			fout.close();
			res = true;
		}

		return res;
	}

	bool ConfigManager::ReloadConfiguration()
	{
		return LoadConfiguration();
	}

	bool ConfigManager::UpdateValue(const std::string key, const double value)
	{
		bool res{ false };

		if (m_root[key])
		{
			m_root[key]["value"] = value;
			if (WriteToFile())
				res = true;
		}

		return res;
	}

	bool ConfigManager::UpdateValue(const std::string key, const std::vector<double> value)
	{
		bool res{ false };

		if (m_root[key])
		{
			m_root[key]["value"] = value;
			if (WriteToFile())
				res = true;
		}

		return res;
	}
}