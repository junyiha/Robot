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
            log->warn("第一层状态数据不正确!!!");
        }
    }
}

void StateMachine::manualStateTransition()
{
    switch (m_esubState)
    {
        case ESubState::eNotReady:
        {
            // 所有手动操作指令
            break;
        }
        case ESubState::eReady:
        {
            readyExecutionCommand();
            break;
        }
        default:
        {
            log->warn("第二层状态不合法!!!");
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
            log->warn("第二层状态不合法!!!");
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
            log->warn("第二层状态不合法!!!");
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
        case ESubState::eTerminationWeld:
        {
            terminationWeldExecutionCommand();
            break;
        }
        default:
        {
            log->warn("第二层状态不合法!!!");
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
            log->warn("第二层状态不合法!!!");
        }
    }
}

void StateMachine::readyExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eManual:
        {
            // 所有手动操作指令

            break;
        }
        case EExecutionCommand::eParallel:
        {
            updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            updateExecutionCommand(EExecutionCommand::eNULL);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void StateMachine::readyToParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eParallel:
        {
            if (m_pdTaskPtr->Parallel())
            {
                updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
                updateExecutionCommand(EExecutionCommand::eNULL);
                break;
            }
            else 
            {
                throw std::bad_exception();
            }
        }
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
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
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
            break;
        }
        case EExecutionCommand::eNULL:
        {
            if (m_pdTaskPtr->CheckFlatness())
            {
                updateTopAndSubState(ETopState::ePositioning, ESubState::eReadyToPositioning);
            }
            else 
            {
                std::bad_exception();
            }            
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
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
            break;
        }
        case EExecutionCommand::eNULL:
        {
            if (m_pdTaskPtr->CheckLine())
            {
                updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
                updateExecutionCommand(EExecutionCommand::eNULL);
            }
            else 
            {
                std::bad_exception();
            }
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
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
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
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
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
        case EExecutionCommand::ePositioning:
        {
            if (m_pdTaskPtr->Positioning())
            {
                updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
                updateExecutionCommand(EExecutionCommand::eNULL);
            }
            else 
            {
                std::bad_exception();
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
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
        case EExecutionCommand::eParallel:
        {
            if (m_pdTaskPtr->Parallel())
            {
                updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            }
            else 
            {
                std::bad_exception();
            }
            break;
        }
        case EExecutionCommand::eMagentOn:
        {
            if (m_pdTaskPtr->MagentOn())
            {
                updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);
                updateExecutionCommand(EExecutionCommand::eNULL);
            }
            else 
            {
                std::bad_exception();
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
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
        case EExecutionCommand::eAutoWeld:
        {
            if (m_pdTaskPtr->AutoDoWeld())
            {
                updateTopAndSubState(ETopState::eDoWeld, ESubState::eDoingWeld);
                updateExecutionCommand(EExecutionCommand::eNULL);
            }
            else 
            {
                std::bad_exception();
            }
            break;
        }
        case EExecutionCommand::eMagentOff:
        {
            break;
        }
        case EExecutionCommand::eTerminate:
        {
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
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::eStopWeld:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
            break;
        }
        case EExecutionCommand::eNULL:
        {
            if (m_pdTaskPtr->CheckAutoDoWeld())
            {
                updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
                updateExecutionCommand(EExecutionCommand::eNULL);
            }
            else 
            {
                std::bad_exception();
            }
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    } 
}

void StateMachine::terminationWeldExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eMagentOff:
        {
            break;
        }
        case EExecutionCommand::eTerminate:
        {
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
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        case EExecutionCommand::ePause:
        {
            break;
        }
        case EExecutionCommand::eNULL:
        {
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
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
        case EExecutionCommand::eQuit:
        {
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
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