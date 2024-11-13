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

		/**
		 * @brief 将YAML数据写入文件.
		 */
		bool WriteToFile();

	public:
		/**
		 * @brief 重新加载参数配置文件.
		 */
		bool ReloadConfiguration();

		/**
		 * @brief 更新示教点.
		 */
		bool UpdateValue(const std::string key, const GP::PositionMap position_map);

	private:
		YAML::Node m_root_ro;
		YAML::Node m_root_rw;
		std::string m_path_ro{ CONFIG_PATH_RO};
		std::string m_path_rw{ CONFIG_PATH_RW};
		std::shared_ptr<spdlog::logger> log;
	};
}
