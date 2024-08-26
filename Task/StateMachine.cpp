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
    switch (etopState)
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
    switch (esubState)
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
    }
}

void StateMachine::parallelStateTransition()
{
    switch (esubState)
    {
        case ESubState::eReadyToParallel:
        {
            readyToParallelExecutionCommand();
            break;
        }
        case ESubState::eDetection:
        {
            detectionExecutionCommand();
            break;
        }
        case ESubState::eMotion:
        {
            motionExecutionCommand();
            break;
        }
    }
}

void StateMachine::positioningStateTransition()
{
    switch (esubState)
    {
        case ESubState::eReadyToPositioning:
        {
            readyToPositioningExecutionCommand();
            break;
        }
        case ESubState::eDetection:
        {
            detectionExecutionCommand();
            break;
        }
        case ESubState::eMotion:
        {
            motionExecutionCommand();
            break;
        }
    }
}

void StateMachine::readyToMagentOnStateTransition()
{
    readyToPositioningExecutionCommand();
}

void StateMachine::doWeldStateTransition()
{
    switch (esubState)
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
    switch (esubState)
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

void StateMachine::notReadyExecutionCommand()
{
    
}

void StateMachine::readyExecutionCommand()
{
    
}

void StateMachine::readyToParallelExecutionCommand()
{
    switch (eexecutionCommand)
    {
        case EExecutionCommand::eParallel:
        {
            if (m_pdTaskPtr->Parallel())
            {
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

void StateMachine::detectionExecutionCommand()
{
    switch (eexecutionCommand)
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

void StateMachine::motionExecutionCommand()
{
    switch (eexecutionCommand)
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
    switch (eexecutionCommand)
    {
        case EExecutionCommand::ePositioning:
        {
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
    switch (eexecutionCommand)
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

void StateMachine::readyToWeldExecutionCommand()
{
    switch (eexecutionCommand)
    {
        case EExecutionCommand::eAutoWeld:
        {
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
    switch (eexecutionCommand)
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
    } 
}

void StateMachine::terminationWeldExecutionCommand()
{
    switch (eexecutionCommand)
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
    switch (eexecutionCommand)
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

void StateMachine::pauseExecutionCommand()
{
    switch (eexecutionCommand)
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
}  // namespace TASK