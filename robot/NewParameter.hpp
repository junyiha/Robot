/**
 * @file NewParameter.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-12-10
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#ifndef NEW_PARAMETER_HPP
#define NEW_PARAMETER_HPP

#include <map>
#include <string>

struct RobotConfig_t
{
    std::string name;
    int min_pos;
    int max_pos;
    int zero_pos;
    float max_velocity;
};

const static std::map<int, RobotConfig_t> RobotConfigMap = {
    {0, {"升降", 0, 250, 0, 7}},
    {1, {"前后", -110, 110, 0, 20}},
    {2, {"左右", -110, 110, 0, 20}},
    {3, {"腰旋转", -95, 95, 0, 10}},
    {4, {"腰俯仰", 75, 105, 90, 3}},
    {5, {"臂俯仰", 790, 1100, 790, 30}},
    {6, {"筒伸缩", 1070, 1700, 1070, 27}},
    {7, {"筒旋转", -30, 30, 0, 5}},
    {8, {"腕俯仰", -100, 90, 0, 10}},
    {9, {"末端升降", 885, 1490, 885, 4}},
    {10, {"末端旋转", 520, 620, 560, 3}}
};

#endif  NEW_PARAMETER_HPP