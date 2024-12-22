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

#include <iostream>
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include "cxxopts.hpp"

struct Args_t
{
    int argc;
    char** argv;
    cxxopts::ParseResult result;
};

#endif  // BASIC_HEADER_HPP