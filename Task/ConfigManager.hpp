/*****************************************************************//**
 * \file   ConfigManager.hpp
 * \brief  处理配置文件
 * 
 * \author anony
 * \date   October 2024
 *********************************************************************/
#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "GVL.h"

namespace Config
{
	class ConfigManager final
	{
	public:
		ConfigManager();
		~ConfigManager();

	private:
		void ParseConfiguration();

	private:
		YAML::Node m_root;
		std::string m_path{ "D:/Robot/config.yaml" };
	};
}
