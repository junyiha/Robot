#include "StateMachine.h"
namespace TASK
{
    
StateMachine::StateMachine(std::shared_ptr<PDTask> pdTaskPtr) : m_pdTaskPtr(pdTaskPtr)
{
    log = spdlog::get("logger");
    log->info("State Machine construction!\n");
}

StateMachine::~StateMachine()
{

}

/////////////////////////////////////////////--状态判断--/////////////////////////////////////////////////

void StateMachine::stateTransition()
{
    switch (m_etopState)
    {
        case ETopState::eManual:
        {
            manualStateTransition();
            break;
        }
        case ETopState::eParallel:
        {
            parallelStateTransition();
            break;
        }
        case ETopState::ePositioning:
        {
            positioningStateTransition();
            break;
        }
        case ETopState::eReadToMagentOn:
        {
            readyToMagentOnStateTransition();
            break;
        }
        case ETopState::eDoWeld:
        {
            doWeldStateTransition();
            break;
        }
        case ETopState::eQuit :
        {
            quitStateTransition();
            break;
        }
        default:
        {
            log->warn("第一层状态数据无效!!!");
        }
    }

    // 将指令置为默认空指令
    updateExecutionCommand();
}

void StateMachine::manualStateTransition()
{
    switch (m_esubState)
    {
        case ESubState::eNotReady:
        {
            notReadyExecutionCommand();
            break;
        }
        case ESubState::eReady:
        {
            readyExecutionCommand();
            break;
        }
        default:
        {
            log->warn("第二层状态数据无效!!!");
        }
    }
}

void StateMachine::parallelStateTransition()
{
    switch (m_esubState)
    {
        case ESubState::eReadyToParallel:
        {
            readyToParallelExecutionCommand();
            break;
        }
        case ESubState::eDetection:
        {
            detectionInParallelExecutionCommand();
            break;
        }
        case ESubState::eMotion:
        {
            motionInParallelExecutionCommand();
            break;
        }
        default:
        {
            log->warn("第二层状态数据无效!!!");
        }
    }
}

void StateMachine::positioningStateTransition()
{
    switch (m_esubState)
    {
        case ESubState::eReadyToPositioning:
        {
            readyToPositioningExecutionCommand();
            break;
        }
        case ESubState::eDetection:
        {
            detectionInPositioningExecutionCommand();
            break;
        }
        case ESubState::eMotion:
        {
            motionInPositioningExecutionCommand();
            break;
        }
        default:
        {
            log->warn("第二层状态数据无效!!!");
        }
    }
}

void StateMachine::readyToMagentOnStateTransition()
{
    readyToMagentOnExecutionCommand();
}

void StateMachine::doWeldStateTransition()
{
    switch (m_esubState)
    {
        case ESubState::eReadyToDoWeld:
        {
            readyToWeldExecutionCommand();
            break;
        }
        case ESubState::eDoingWeld:
        {
            doingWeldExecutionCommand();
            break;
        }
        case ESubState::eStopWeld:
        {
            stopWeldExecutionCommand();
            break;
        }
        default:
        {
            log->warn("第二层状态数据无效!!!");
        }
    }
}

void StateMachine::quitStateTransition()
{
    switch (m_esubState)
    {
        case ESubState::eQuiting:
        {
            quitingExecutionCommand();
            break;
        }
        case ESubState::ePause:
        {
            pauseExecutionCommand();
            break;
        }
        default:
        {
            log->warn("第二层状态数据无效!!!");
        }
    }
}

/////////////////////////////////////////////--指令判断--/////////////////////////////////////////////////

void StateMachine::readyExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eManual:
        {
            log->info("所有手动操作指令,等待手动触发...\n") ;
            break;
        }
        case EExecutionCommand::eParallel:
        {
            updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::notReadyExecutionCommand()
{
    log->warn("设备有错误，或机器人移动。此时执行 所有手动操作指令");
}

void StateMachine::readyToParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("调平-待调平下 空指令，不执行任何操作");
            break;
        }
        case EExecutionCommand::eParallel:
        {
            log->info("调平指令触发，状态跳转至: 调平--检测");
            updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("待调平状态下的暂停指令，不执行任何动作");
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::detectionInParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("调用函数，根据激光数值判断壁面状态");
            auto detectionResult = m_pdTaskPtr->CheckFlatness();
            switch (detectionResult)
            {
                case EDetectionInParallelResult::eDeviationIsLessThanThreshold:
                {
                    updateTopAndSubState(ETopState::ePositioning, ESubState::eReadyToPositioning);
                    break;
                }
                case EDetectionInParallelResult::eDistanceMeetsRequirement:
                {
                    updateTopAndSubState(ETopState::eParallel, ESubState::eMotion);
                    break;
                }
                case EDetectionInParallelResult::eNoWallDetected:
                {
                    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
                    break;
                }
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            updateTopAndSubState(ETopState::eParallel, ESubState::eReadyToParallel);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }    
}

void StateMachine::motionInParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("读激光反馈，反馈异常时停止运动，否则继续运动");
            auto laserResult = m_pdTaskPtr->CheckLaser();
            if (laserResult)
            {
                log->info("反馈正常， stanstill，状态跳转: 调平--检测");
                updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            }
            else 
            {
                log->info("反馈异常，末端激光偏差过大，状态跳转: 手动");
                updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("暂停指令，停止运动，状态跳转: 调平--待调平");
            updateTopAndSubState(ETopState::eParallel, ESubState::eReadyToParallel);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::readyToPositioningExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("定位--待定位状态下 空指令，不执行任何操作");
            break;
        }
        case EExecutionCommand::ePositioning:
        {
            log->info("定位指令，状态跳转: 定位--检测");
            updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("定位--待定位状态下，暂停指令不执行任何操作");
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::detectionInPositioningExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("调用函数，计算边线偏差，根据偏差判断是否完成调整，继续调整，停止调整");
            auto detectResult = m_pdTaskPtr->CheckLine();
            switch (detectResult)
            {
                case EDetectionInPositioningResult::eDeviationIsLessThanThreshold:
                {
                    updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
                    break;
                }
                case EDetectionInPositioningResult::eEndAdjustmentDataIsValid:
                {
                    updateTopAndSubState(ETopState::ePositioning, ESubState::eMotion);
                    break;
                }
                case EDetectionInPositioningResult::eDataIsInvalid:
                {
                    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
                    break;
                }
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("定位--检测状态下，暂停指令不执行任何操作");
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }        
}

void StateMachine::motionInPositioningExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("末端运动。末端运动完成，状态跳转: 定位--检测");
            updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("停止运动，状态跳转: 定位--待定位");
            updateTopAndSubState(ETopState::ePositioning, ESubState::eReadyToPositioning);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::readyToMagentOnExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("待吸合空指令，不执行任何操作");
            break;
        }
        case EExecutionCommand::eParallel:
        {
            log->info("调平指令，状态跳转: 调平--检测");
            updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eMagentOn:
        {
            log->info("磁铁吸合指令，状态跳转: 碰钉--待碰钉");
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);
            break;
        }
        case EExecutionCommand::eQuit:
        {
            updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::readyToWeldExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("碰钉-待碰钉状态下 空指令，不执行任何操作");
            break;
        }
        case EExecutionCommand::eAutoWeld:
        {
            log->info("自动碰钉指令，状态跳转: 碰钉-碰钉中");
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eDoingWeld);
            break;
        }
        case EExecutionCommand::eMagentOff:
        {
            log->info("磁铁脱开，状态跳转: 待吸合");
            updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::doingWeldExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("碰钉--碰钉中 空指令，执行自动碰钉，完成碰钉后状态跳转: 退出--退出中");
            updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        case EExecutionCommand::eStopWeld:
        {
            log->info("停止碰钉，状态跳转: 碰钉--停止");
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eStopWeld);
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("暂停碰钉，状态跳转: 碰钉--待碰钉");
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    } 
}

void StateMachine::stopWeldExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("碰钉--停止状态下 空指令，不做任何操作");
            break;
        }
        case EExecutionCommand::eMagentOff:
        {
            log->info("磁铁脱开，状态跳转: 待吸合");
            updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::quitingExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("退出--退出中状态下 空指令，移动到退出点，到达退出点后状态跳转: 手动");
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("停止运动，状态跳转: 退出--暂停");
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::pauseExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("退出-暂停状态下 空指令，不执行任何操作");
            break;
        }
        case EExecutionCommand::eQuit:
        {
            log->info("退出指令，状态跳转: 退出--退出中");
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateMachine::terminateCommand()
{
    log->warn("停止运行，并进行状态跳转\n");
    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
}

void StateMachine::updateTopAndSubState(ETopState topState, ESubState subState)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_etopState = topState;
    m_esubState = subState;
}

void StateMachine::updateExecutionCommand(EExecutionCommand executionCommand)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_eexecutionCommand = executionCommand;
}

std::string StateMachine::getCurrentStateString()
{
    std::string stateString;
    auto itTopState = TopStateStringMap.find(m_etopState);
    stateString = itTopState->second;

    stateString += "--";

    auto itSubState = SubStateStringMap.find(m_esubState);
    stateString += itSubState->second;

    return stateString;
}

std::string StateMachine::getCurrentExecutionCommandString()
{
    auto it = ExecutionCommandStringMap.find(m_eexecutionCommand);

    return it->second;
}

}  // namespace TASK