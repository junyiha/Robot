/*****************************************************************//**
 * \file   TaskExternal.hpp
 * \brief  
 * 
 * \author anony
 * \date   October 2024
 *********************************************************************/
#pragma once

enum class ETopState
{
    eManual = 0,        // 手动
    eParallel,          // 调平
    ePositioning,       // 定位
    eReadToMagentOn,    // 待吸合
    eDoWeld,            // 碰钉
    eQuit               // 退出
};

enum class ESubState
{
    eNULL = 0,          // 空状态
    eNotReady = 0,      // 未就绪
    eReady,             // 就绪
    eMotion,            // 运动

    eReadyToParallel,   // 待调平
    eDetection,         // 检测

    eReadyToPositioning,// 待定位

    eReadyToDoWeld,     // 待碰钉
    eDoingWeld,         // 碰钉中
    eStopWeld,          // 碰钉停止

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
    eDataIsInvalid,                         // 数据非法
};

enum class ActionKey
{
    Grind_MovorOff1 = 0,
    Grind_OnorDown1 = 40,
    Grind_Up = 60,
    Grind_OnorDown2 = 160,
    Grind_MovorOff2 = 180,
    Weld_MovorDwon = 200,
    Weld_Fix = 240,
    Weld_Up = 280,
    Weld_On = 320,
    Weld_Down = 360,
    InitAction = 400,
    End = 405
};

