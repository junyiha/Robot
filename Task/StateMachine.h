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
#include <mutex>
#include <functional>
#include <map>

#include "spdlog/spdlog.h"
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
    eNULL = 0,          // 空状态
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
    eNULL = 0,          // 空指令
    eManual = 0,        // 手动指令
    eParallel,          // 调平
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
    /**
     * @brief 状态转换函数
     * 
     */
    void stateTransition();

private:
    /**
     * @brief 手动状态
     * 
     */
    void manualStateTransition();

    /**
     * @brief 调平状态
     * 
     */
    void parallelStateTransition();

    /**
     * @brief 定位状态
     * 
     */
    void positioningStateTransition();

    /**
     * @brief 待吸合状态
     * 
     */
    void readyToMagentOnStateTransition();

    /**
     * @brief 碰钉状态
     * 
     */
    void doWeldStateTransition();

    /**
     * @brief 退出状态
     * 
     */
    void quitStateTransition();


    /**
     * @brief 手动指令
     * 
     */
    void readyExecutionCommand();

    /**
     * @brief 调平--待调平状态下，可执行指令
     * 
     */
    void readyToParallelExecutionCommand();

    /**
     * @brief 调平--检测状态下，可执行指令
     * 
     */
    void detectionInParallelExecutionCommand();

    /**
     * @brief 定位--检测状态下，可执行指令
     * 
     */
    void detectionInPositioningExecutionCommand();

    /**
     * @brief 调平--运动状态下，可执行指令
     * 
     */
    void motionInParallelExecutionCommand();

    /**
     * @brief 定位--运动状态下，可执行指令
     * 
     */
    void motionInPositioningExecutionCommand();

    /**
     * @brief 定位--待定位状态下，可执行指令
     * 
     */
    void readyToPositioningExecutionCommand();

    /**
     * @brief 待吸合状态下，可执行指令
     * 
     */
    void readyToMagentOnExecutionCommand();

    /**
     * @brief 碰钉--待碰钉状态下，可执行指令
     * 
     */
    void readyToWeldExecutionCommand();

    /**
     * @brief 碰钉--碰钉中状态下，可执行指令
     * 
     */
    void doingWeldExecutionCommand();

    /**
     * @brief 碰钉--停止状态下，可执行指令
     * 
     */
    void terminationWeldExecutionCommand();

    /**
     * @brief 退出--退出中状态下，可执行指令
     * 
     */
    void quitingExecutionCommand();

    /**
     * @brief 退出--暂停状态下，可执行指令
     * 
     */
    void pauseExecutionCommand();

public:
    /**
     * @brief 更新第一层和第二层状态(线程安全)
     * 
     * @param topState 
     * @param subState 
     */
    void updateTopAndSubState(ETopState topState, ESubState subState);

    /**
     * @brief 更新执行指令(线程安全)
     * 
     * @param executionCommand 
     */
    void updateExecutionCommand(EExecutionCommand executionCommand);

    /**
     * @brief 获取当前状态字符串，格式: 第一层状态--第二层状态
     * 
     * @return std::string 
     */
    std::string getCurrentStateString();

    /**
     * @brief 获取当前执行指令字符串
     * 
     * @return std::string 
     */
    std::string getCurrentExecutionCommandString();

private:
    ETopState m_etopState{ETopState::eManual};
    ESubState m_esubState{ESubState::eReady};
    EExecutionCommand m_eexecutionCommand{EExecutionCommand::eNULL};

private:
    std::mutex m_mutex;
    std::shared_ptr<PDTask> m_pdTaskPtr;
    std::shared_ptr<spdlog::logger> log;

    std::map<ETopState, std::string> TopStateStringMap 
    {
        {ETopState::eManual, "手动"},
        {ETopState::eParallel, "调平"},
        {ETopState::ePositioning, "定位"},
        {ETopState::eReadToMagentOn, "待吸合"},
        {ETopState::eDoWeld, "碰钉"},
        {ETopState::eQuit, "退出"}
    };
    std::map<ESubState, std::string> SubStateStringMap
    {
        {ESubState::eNULL, "空状态"},
        {ESubState::eNotReady, "未就绪"},
        {ESubState::eReady, "就绪"},
        {ESubState::eMotion, "运动"},
        {ESubState::eReadyToParallel, "待调平"},
        {ESubState::eDetection, "检测"},
        {ESubState::eReadyToPositioning, "待定位"},
        {ESubState::eReadyToDoWeld, "待碰钉"},
        {ESubState::eDoingWeld, "碰钉中"},
        {ESubState::eTerminationWeld, "碰钉终止"},
        {ESubState::eQuiting, "退出中"},
        {ESubState::ePause, "暂停"}
    };
    std::map<EExecutionCommand, std::string> ExecutionCommandStringMap
    {
        {EExecutionCommand::eNULL, "空指令"},
        {EExecutionCommand::eManual, "手动指令"},
        {EExecutionCommand::eParallel, "调平"},
        {EExecutionCommand::eTerminate, "终止"},
        {EExecutionCommand::ePause, "暂停"},
        {EExecutionCommand::ePositioning, "定位"},
        {EExecutionCommand::eMagentOn, "吸合"},
        {EExecutionCommand::eQuit, "退出"},
        {EExecutionCommand::eAutoWeld, "自动碰钉"},
        {EExecutionCommand::eMagentOff, "脱开"},
        {EExecutionCommand::eStopWeld, "停止碰钉"}
    };
};

}  // namespace TASK

#endif // STATE_MACHINE_H