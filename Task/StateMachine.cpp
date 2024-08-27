#include "StateMachine.h"
namespace TASK
{
    
StateMachine::StateMachine(std::shared_ptr<PDTask> pdTaskPtr) : m_pdTaskPtr(pdTaskPtr)
{

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
    }
}

void StateMachine::manualStateTransition()
{
    switch (m_esubState)
    {
        case ESubState::eNotReady:
        {
            break;
        }
        case ESubState::eReady:
        {
            break;
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
                updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eReadyToPositioning);
            }
            else 
            {
                std::bad_exception();
            }
        }
        default:
        {

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
            updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
        }
        default:
        {

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