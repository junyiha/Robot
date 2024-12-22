/****************************************************************
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
#include <map>
#include "spdlog/spdlog.h"
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

		void ParseConfigurationRW();
		void ParseConfigurationRO();

		bool WriteToFile();

	public:
		/**
		 * @brief 重新加载参数配置文件.
		 */
		bool ReloadConfiguration();

		/**
		 * @brief 更新指定参数(浮点数).
		 */
		bool UpdateValue(const std::string key, const GP::PositionMap position_map);

	private:
		YAML::Node m_root_rw;
		YAML::Node m_root_ro;
		std::string m_path_rw{ ROOT_PATH };
		std::string m_path_ro{ ROOT_PATH };
		std::shared_ptr<spdlog::logger> log;
	};
}
