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

		bool WriteToFile();

	public:
		/**
		 * @brief ���¼��ز��������ļ�.
		 */
		bool ReloadConfiguration();

		/**
		 * @brief ����ָ������(������).
		 */
		bool UpdateValue(const std::string key, const double value);

		/**
		 * @brief ����ָ������(����).
		 */
		bool UpdateValue(const std::string key, const std::vector<double> value);

	private:
		YAML::Node m_root;
		std::string m_path{ "D:/Robot/config.yaml" };
		std::shared_ptr<spdlog::logger> log;
	};
}
