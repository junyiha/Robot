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

enum class EDetectionInParallelResult
{
    eDeviationIsLessThanThreshold = 0,     // 激光传感器偏差小于阈值
    eDistanceMeetsRequirement,             // 板壁距离满足调整要求
    eNoWallDetected                        // 未检测到壁面
};

enum class EDetectionInPositioningResult
{
    eDeviationIsLessThanThreshold = 0,     // 边线偏差小于阈值
    eEndAdjustmentDataIsValid,             // 末端调整数据合法
    eDataIsInvalid,                         // 数据非法

};

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
    EDetectionInParallelResult CheckFlatness();

    /**
     * @brief 读取激光反馈数据
     * 
     * @return true 
     * @return false 
     */
    bool CheckLaser();

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
    EDetectionInPositioningResult CheckLine();

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