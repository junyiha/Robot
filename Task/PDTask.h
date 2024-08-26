/**
 * @file PDTask.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once
#ifndef PD_TASK_H
#define PD_TASK_H

#include <iostream>
#include <thread>
#include <chrono>

namespace TASK
{

class PDTask
{
public:
    PDTask();
    ~PDTask();

    bool Parallel();
};

}  // namespace TASK

#endif  // PD_TASK_H