/**
 * @file TaskExternal.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-09-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once
#ifndef TASK_EXTERNAL_H
#define TASK_EXTERNAL_H


// 测试状态机的宏指令
#ifndef TEST_TASK_STATEMACHINE_
#define TEST_TASK_STATEMACHINE_
#endif

enum class ETopState
{
    eManual = 0,        // 手动
    eParallel,          // 调平
    ePositioning,       // 定位
    eReadToMagentOn,    // 待吸合
    eDoWeld,            // 碰钉
    eFitBoard,          // 贴合
    eQuit               // 退出
};

enum class ESubState
{
    eNULL = 0,          // 空状态
    eNotReady,          // 未就绪
    eReady,             // 就绪
    eMotion,            // 运动

    eReadyToParallel,   // 待调平
    eDetection,         // 检测

    eReadyToPositioning,// 待定位
    
    eReadyToDoWeld,     // 待碰钉
    eDoingWeld,         // 碰钉中
    eStopWeld,          // 碰钉停止

    eReadyToFitBoard,   // 待贴合
    eFitBoardFinished,  // 贴合完成

    eQuiting,           // 退出中
    ePause              // 暂停
};

enum class EExecutionCommand
{
    eNULL = 0,          // 空指令
    eManual,            // 手动指令
    eParallel,          // 调平
    eTerminate,         // 终止
    ePause,             // 暂停
    ePositioning,       // 定位
    eMagentOn,          // 吸合
    eQuit,              // 退出
    eAutoWeld,          // 自动碰钉
    eFitBoard,          // 贴合
    eMagentOff,         // 脱开
    eStopWeld,          // 停止碰钉
    eSideline,          // 对齐边线 == 定位(ePositioning)
    eLift,              // 举升 (手动指令中的)
    eAddNail,           // 放钉 (手动指令中的)
    eStop,              // 停止 (手动指令中的)
    eCrashStop,         // 急停 (手动指令中的)
};

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
    eDataIsInvalid,                        // 数据非法
};

#endif  // TASK_EXTERNAL_H