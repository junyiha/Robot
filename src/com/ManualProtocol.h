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
    bool StopCommand; // 界面停止
    bool HaltCommand; // 急停指令

    // 车运动指令
    double VechVel;    // 底盘行走: 前进/后退 23~24
    double RotateVel;  // 底盘行走: 差速转向 25~26
    bool bVechFlag;    // 底盘行走: 前进/后退标志位 int16 = 0? false: true
    bool bRotateFlag;  // 底盘行走: 差速转向标志位 int16 = 0? false: true
    double VechDirect; // 舵轮: 舵轮控制 9~10

    // 机械臂运动指令
    std::vector<double> LinkMove{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // 机器人关节角/末端位置 11~22
    bool bLinkMoveFlag;                                         // 机器人运动标志位 all int16 = 0 ? false:true
    bool bEndMove;                                              // 模式: 切换推杆模式 true--末端 | false--单轴 保开1 buffer[8] bit0,bit1

    // 设置指令[
    int Ready; // 0: 不动, 1: 举升, 2: 放下 buffer[8] bit2,bit3

    // 任务指令
    enum ETaskIndex // 任务: 执行任务
    {
        None = 0,
        SecondPush = 1,
        Quit = 2,
        Parallel = 4,
        SecondQuit = 8,
        Positioning = 16,
        Pause = 32,
        FitBoard = 64,
        Terminate = 128,
        MagentOff = 9, // 暂时保留，避免编译问题，后续删除
        DoWeld = 65,   // 暂时保留，避免编译问题，后续删除
        MagentOn = 3   // 暂时保留，避免编译问题，后续删除
    };

    int TaskIndex; // = buff[7]
};

#endif // MANUAL_PROTOCOL_H