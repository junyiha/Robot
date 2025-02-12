/**
 * @file basic_header.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-12-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#ifndef BASIC_HEADER_HPP
#define BASIC_HEADER_HPP

extern "C"
{
#include <conio.h>
}

#include <clocale>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <bitset>
#include <exception>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include "cxxopts.hpp"

#include "task/GlobalParameter.hpp"

struct Args_t
{
    int argc;
    char** argv;
    cxxopts::ParseResult result;
};

#endif  // BASIC_HEADER_HPP