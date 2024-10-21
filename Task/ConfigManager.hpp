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
		/**
		 * @brief 加载参数配置文件.
		 */
		bool LoadConfiguration();

		/**
		 * @brief 解析参数配置文件，初始化全局参数.
		 */
		void ParseConfiguration();

	public:
		/**
		 * 重新加载参数配置文件.
		 */
		bool ReloadConfiguration();

	private:
		YAML::Node m_root;
		std::string m_path{ "D:/Robot/config.yaml" };
	};
}
