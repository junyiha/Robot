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

	public:
		/**
		 * ���¼��ز��������ļ�.
		 */
		bool ReloadConfiguration();

	private:
		YAML::Node m_root;
		std::string m_path{ "D:/Robot/config.yaml" };
	};
}
