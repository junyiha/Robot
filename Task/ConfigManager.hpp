/*****************************************************************//**
 * \file   ConfigManager.hpp
 * \brief  ���������ļ�
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
		 * @brief ���ز��������ļ�.
		 */
		bool LoadConfiguration();

		/**
		 * @brief �������������ļ�����ʼ��ȫ�ֲ���.
		 */
		void ParseConfiguration();

		/**
		 * @brief ��YAML����д���ļ�.
		 */
		bool WriteToFile();

	public:
		/**
		 * @brief ���¼��ز��������ļ�.
		 */
		bool ReloadConfiguration();

		/**
		 * @brief ����ʾ�̵�.
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
