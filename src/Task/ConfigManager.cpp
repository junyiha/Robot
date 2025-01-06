#include "ConfigManager.hpp"

namespace Config
{
	ConfigManager::ConfigManager()
	{
		log = spdlog::get("logger");

		m_path_rw += "configurations/config_rw.yaml";
		m_path_ro += "configurations/config_ro.yaml";

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
			m_root_rw = YAML::LoadFile(m_path_rw);
			m_root_ro = YAML::LoadFile(m_path_ro);
			ParseConfiguration();
		}
		catch (YAML::BadFile)
		{
			log->error("Invalid configuration file path: {} or {}", m_path_rw, m_path_ro);
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
		ParseConfigurationRO();
		ParseConfigurationRW();
	}

	void ConfigManager::ParseConfigurationRW()
	{
		for (auto& it : m_root_rw["position_map"])
		{
			auto work_scenario = static_cast<GP::WorkingScenario>(it[1].as<int>());
			auto position_type = static_cast<GP::PositionType>(it[2].as<int>());

			GP::PositionData position_data;
			position_data.brief = it[0].as<std::string>();
			position_data.value = it[3].as<std::vector<double>>();

			GP::Position_Map[{work_scenario, position_type}] = position_data;
		}
	}

	void ConfigManager::ParseConfigurationRO()
	{
		GP::End_Vel_Limit = m_root_ro["end_vel_limit"]["value"].as<std::vector<double>>();

		GP::Robot_IP = m_root_ro["robot_ip"]["value"].as<std::string>();
		GP::Robot_Port = m_root_ro["robot_port"]["value"].as<std::size_t>();

		GP::IOA_IP = m_root_ro["IOA_ip"]["value"].as<std::string>();
		GP::IOA_Port = m_root_ro["IOA_port"]["value"].as<std::size_t>();

		GP::CYLINDER_INDEX = m_root_ro["CYLINDER_INDEX"]["value"].as<uint>();
		GP::STEER_LEFT_INDEX = m_root_ro["STEER_LEFT_INDEX"]["value"].as<uint>();
		GP::STEER_RIGHT_INDEX = m_root_ro["STEER_RIGHT_INDEX"]["value"].as<uint>();
		GP::WHEEL_LEFT_INDEX = m_root_ro["WHEEL_LEFT_INDEX"]["value"].as<uint>();
		GP::WHEEL_RIGHT_INDEX = m_root_ro["WHEEL_RIGHT_INDEX"]["value"].as<uint>();
		GP::TOOL_LIFTING = m_root_ro["TOOL_LIFTING"]["value"].as<uint>();

		GP::velLine = m_root_ro["velLine"]["value"].as<double>();
		GP::velRotate = m_root_ro["velRotate"]["value"].as<double>() / 57.3;

		GP::Lift_Distance_In_Parallel = m_root_ro["Lift_Distance_In_Parallel"]["value"].as<double>();
		GP::Max_Deviation_In_Parallel = m_root_ro["Max_Deviation_In_Parallel"]["value"].as<double>();
		GP::Min_Deviation_In_Parallel = m_root_ro["Min_Deviation_In_Parallel"]["value"].as<double>();
		GP::Distance_work = m_root_ro["Distance_work"]["value"].as<double>();
		GP::Lift_Distance_In_FitBoard = m_root_ro["Lift_Distance_In_FitBoard"]["value"].as<double>();
		GP::Max_Deviation_In_FitBoard = m_root_ro["Max_Deviation_In_FitBoard"]["value"].as<double>();
		GP::Min_Deviation_In_FitBoard = m_root_ro["Min_Deviation_In_FitBoard"]["value"].as<double>();
		GP::Line_Deviation_Threshold = m_root_ro["Line_Deviation_Threshold"]["value"].as<double>();

		GP::Second_Push_Distance = m_root_ro["Second_Push_Distance"]["value"].as<double>();
		GP::Second_Quit_Distance = m_root_ro["Second_Quit_Distance"]["value"].as<double>();
	}

	bool ConfigManager::WriteToFile()
	{
		bool res{ false };

		std::ofstream fout(m_path_rw);
		if (fout)
		{
			fout << m_root_rw;
			fout.close();
			res = true;
		}

		return res;
	}

	bool ConfigManager::ReloadConfiguration()
	{
		return LoadConfiguration();
	}

	bool ConfigManager::UpdateValue(const std::string key, const GP::PositionMap position_map)
	{
		bool res{ false };
		YAML::Node node;
		for (auto& it : position_map)
		{
			YAML::Node temp_node;
			temp_node.push_back(it.second.brief);
			temp_node.push_back(static_cast<int>(it.first.first));
			temp_node.push_back(static_cast<int>(it.first.second));
			temp_node.push_back(it.second.value);
			node.push_back(temp_node);
		}
		m_root_rw[key] = node;
		if (WriteToFile())
			res = true;

		return res;
	}
}