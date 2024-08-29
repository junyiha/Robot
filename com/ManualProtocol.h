/**
 * @file ManualProtocol.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once 
#ifndef MANUAL_PROTOCOL_H   
#define MANUAL_PROTOCOL_H

#include <vector>
#include <cmath>

struct stManualOperator
{
    // 急停指令
    bool StopCommand;   // 界面停止
    bool HaltCommand;   // 急停指令

    // 车运动指令
    double VechVel;     // 底盘行走: 前进/后退
    double RotateVel;   // 底盘行走: 差速转向
    bool bVechFlag;     // 底盘行走: 前进/后退标志位
    bool bRotateFlag;   // 底盘行走: 差速转向标志位
    double VechDirect;  // 舵轮: 舵轮控制

    // 机械臂运动指令
    std::vector<double> LinkMove{6, 0.0};   // 机器人关节角/末端位置
    bool bLinkMoveFlag;                     // 机器人运动标志位
    bool bEndMove;      // 模式: 切换推杆模式 true--末端 | false--单轴

    // 设置指令
    int Ready;          // 0: 不动, 1: 举升, 2: 放下

    // 任务指令
    enum ETaskIndex      // 任务: 执行任务
    {
        Parallel = 0,
        Positioning,
        DoWeld,
        MagentOn,
        MagentOff,
        Quit,
        Pause,
        Terminate
    };

    int TaskIndex;
};


#endif  // MANUAL_PROTOCOL_H