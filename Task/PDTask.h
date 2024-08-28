/**
 * @file PDTask.h
 * @author zhangjunyi (you@domain.com)
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

    /**
     * @brief 调平
     * 
     * @return true 
     * @return false 
     */
    bool Parallel();

    /**
     * @brief 检测平整度
     * 
     * @return true 
     * @return false 
     */
    bool CheckFlatness();

    /**
     * @brief 定位
     * 
     * @return true 
     * @return false 
     */
    bool Positioning();

    /**
     * @brief 检测边线
     * 
     * @return true 
     * @return false 
     */
    bool CheckLine();

    /**
     * @brief 吸合
     * 
     * @return true 
     * @return false 
     */
    bool MagentOn();

    /**
     * @brief 自动碰钉
     * 
     * @return true 
     * @return false 
     */
    bool AutoDoWeld();


    /**
     * @brief 检查自动碰钉任务是否完成
     * 
     * @return true 
     * @return false 
     */
    bool CheckAutoDoWeld();
};

}  // namespace TASK

#endif  // PD_TASK_H