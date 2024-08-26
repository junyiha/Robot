/**
 * @file StateMachine.h
 * @author zhangjunyi (you@domain.com)
 * @brief state machine
 * @version 0.1
 * @date 2024-08-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#pragma once
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include <exception>
#include "PDTask.h"

namespace TASK
{

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
    eNotReady = 0,      // 未就绪
    eReady,             // 就绪
    eMotion,            // 运动

    eReadyToParallel,   // 待调平
    eDetection,         // 检测

    eReadyToPositioning,// 待定位
    
    eReadyToDoWeld,     // 待碰钉
    eDoingWeld,         // 碰钉中
    eTerminationWeld,   // 碰钉终止

    eQuiting,           // 退出中
    ePause              // 暂停
};

enum class EExecutionCommand
{
    eParallel = 0,      // 调平
    eTerminate,         // 终止
    ePause,             // 暂停
    ePositioning,       // 定位
    eMagentOn,          // 吸合
    eQuit,              // 退出
    eAutoWeld,          // 自动碰钉
    eMagentOff,         // 脱开
    eStopWeld           // 停止碰钉
};

class StateMachine
{
public:
    StateMachine() = delete;
    StateMachine(std::shared_ptr<PDTask> pdTaskPtr);
    ~StateMachine();
    void stateTransition();

private:
    void manualStateTransition();
    void parallelStateTransition();
    void positioningStateTransition();
    void readyToMagentOnStateTransition();
    void doWeldStateTransition();
    void quitStateTransition();

    void notReadyExecutionCommand();
    void readyExecutionCommand();
    void readyToParallelExecutionCommand();
    void detectionExecutionCommand();
    void motionExecutionCommand();
    void readyToPositioningExecutionCommand();
    void readyToMagentOnExecutionCommand();
    void readyToWeldExecutionCommand();
    void doingWeldExecutionCommand();
    void terminationWeldExecutionCommand();
    void quitingExecutionCommand();
    void pauseExecutionCommand();

public:
    ETopState etopState{ETopState::eManual};
    ESubState esubState{ESubState::eNotReady};
    EExecutionCommand eexecutionCommand{EExecutionCommand::ePause};

private:
    std::shared_ptr<PDTask> m_pdTaskPtr;
};

}  // namespace TASK

#endif // STATE_MACHINE_H
