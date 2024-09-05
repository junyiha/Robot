/**
 * @file TaskExternal.cpp
 * @author your name (you@domain.com)
 * @brief 状态机相关源代码
 * @version 0.1
 * @date 2024-08-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "Task.h"

void CTask::stateTransition()
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

void CTask::manualStateTransition()
{
    //1. 判断机器人是否处于就绪状态 
    bool robotReady = (m_LinkStatus.eLinkActState == eLINK_STANDSTILL) && (m_Robot->isJointReached(Postion_Home_qv));
    robotReady = true;  // 状态机离线调试(2024.09.03)
    if (robotReady)
    {
        //2. 如果就绪，判断是否要执行调平指令
        if(m_eexecutionCommand == EExecutionCommand::eParallel)
        {
            if(robotReady)
            {
                updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            }
            else
            {
                log->warn("机器人未就绪，无法执行调平指令");
            }
            return;
        }
    }

    //3. 执行手动操作指令 -- 遥控器处理函数--》遥控器对接。
    Manual();
}

void CTask::parallelStateTransition()
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

void CTask::positioningStateTransition()
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

void CTask::readyToMagentOnStateTransition()
{
    readyToMagentOnExecutionCommand();
}

void CTask::doWeldStateTransition()
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

void CTask::quitStateTransition()
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

void CTask::readyExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eManual:
        {
            // log->info("所有手动操作指令,等待手动触发...\n") ;
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

void CTask::notReadyExecutionCommand()
{
    log->warn("设备有错误，或机器人移动。此时执行 所有手动操作指令");
}

void CTask::readyToParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // log->info("调平-待调平下 空指令，不执行任何操作");
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
            log->info("待调平状态下的终止指令，进入手动状态");
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            // log->info("待调平状态下的暂停指令，不执行任何动作");
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::detectionInParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("调用函数，根据激光数值判断壁面状态");
            QVector<double>  laserDistance = m_Comm->getLasersDistance();

            auto detectionResult = CheckParallelStateDecorator(laserDistance);
            log->info("laser distance: {},{},{},{}", laserDistance[0],laserDistance[1],laserDistance[2],laserDistance[3]);
            switch (detectionResult)
            {
                case EDetectionInParallelResult::eDeviationIsLessThanThreshold:
                {
                    log->info("updateTopAndSubState(ETopState::ePositioning, ESubState::eReadyToPositioning);");
                    updateTopAndSubState(ETopState::ePositioning, ESubState::eReadyToPositioning);
                    break;
                }
                case EDetectionInParallelResult::eDistanceMeetsRequirement:
                {
                    log->info("updateTopAndSubState(ETopState::eParallel, ESubState::eMotion);");
                    updateTopAndSubState(ETopState::eParallel, ESubState::eMotion);
                    break;
                }
                case EDetectionInParallelResult::eNoWallDetected:
                {
                    log->info("updateTopAndSubState(ETopState::eManual, ESubState::eReady);");
                    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
                    break;
                }
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            log->info("待调平状态下的终止指令，进入手动状态");
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

void CTask::motionInParallelExecutionCommand()
{
    static QVector<double> last_tar_position{0, 0, 0, 0, 0, 0}; 
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            QVector<double> LaserDistance = m_Comm->getLasersDistance();

            auto result = CheckParallelStateDecorator(LaserDistance);
            if (result == EDetectionInParallelResult::eNoWallDetected)
            {
                log->warn("反馈异常，末端激光偏差过大，状态跳转: 手动");
                updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            }
            else if (result == EDetectionInParallelResult::eDeviationIsLessThanThreshold)
            { 
                log->info("完成调平运动，再次进入调平检测");
                m_Robot->setLinkHalt();
                updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);

            }
            else
            {
                log->warn("激光测量数据满足调整条件，控制机器人调平运动");
            
                m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
                m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
                m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
                m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];

                // 计算偏差，控制机器人运动
                QVector<Eigen::Matrix4d> Dev_RT =  CMeasure::calPoseDeviation(m_stMeasuredata);
                QVector<double> tar_position = m_Robot->getTargetPose(Dev_RT[0]);

                double tar_pos[6] ;
                for(int i=0;i<6;i++)
                {
                    tar_pos[i] = tar_position[i];
                }
                log->info("m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);\ntar_pos:{},{},{},{},{},{}", 
                          tar_pos[0],tar_pos[0],tar_pos[0],tar_pos[0],tar_pos[0],tar_pos[0]);
                // m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);
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
            m_Robot->setLinkHalt();
            updateTopAndSubState(ETopState::eParallel, ESubState::eReadyToParallel);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::readyToPositioningExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // log->info("定位--待定位状态下 空指令，不执行任何操作");
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

void CTask::detectionInPositioningExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            log->info("调用函数，计算边线偏差，根据偏差判断是否完成调整，继续调整，停止调整");
            updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);  // 状态机离线调试(2024.09.03)
            break;

            //调用视觉函数
            auto vis_res = m_vision->getVisResult();
            if(!vis_res.status) 
            { // 视觉检测无数据，等待下一个周期
                break;
            }
            
            std::copy(std::begin(vis_res.stData.m_LineDistance) , std::end(vis_res.stData.m_LineDistance), m_stMeasuredata.m_LineDistance);
            std::copy(std::begin(vis_res.stData.m_bLineDistance), std::end(vis_res.stData.m_bLineDistance), m_stMeasuredata.m_bLineDistance);

            auto detectResult = CheckPositionStateDecorator();
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

void CTask::motionInPositioningExecutionCommand()
{
    static QVector<double> last_tar_position{0, 0, 0, 0, 0, 0}; 
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            QVector<double> LaserDistance = m_Comm->getLasersDistance();

            if (CheckParallelStateDecorator(LaserDistance) == EDetectionInParallelResult::eNoWallDetected)
            {
                log->warn("反馈异常，末端激光偏差过大，状态跳转: 手动");
                updateTopAndSubState(ETopState::eManual, ESubState::eReady);
                break;
            }

            QVector<Eigen::Matrix4d> Dev_RT =  CMeasure::calPoseDeviation(m_stMeasuredata);
            QVector<double> tar_position = m_Robot->getTargetPose(Dev_RT[3]);

            double tar_pos[6] ;
            for(int i=0;i<6;i++)
            {
                tar_pos[i] = tar_position[i];
            }
            m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);

            if (m_LinkStatus.eLinkActState == eLINK_STANDSTILL && 
                m_Robot->isEndReached(tar_position))
            {
                log->warn("定位运动完成，状态跳转: 定位--检测");
                updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
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

void CTask::readyToMagentOnExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // log->info("待吸合空指令，不执行任何操作");
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
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);  // 状态机离线调试(2024.09.03)
            if (doMagentOn())
            {
                updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);
            }
            break;
        }
        case EExecutionCommand::eQuit:
        {
            if (doMagentOff())
            {
                updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            if (doMagentOff())
            {
                terminateCommand();
            }
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::readyToWeldExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // log->info("碰钉-待碰钉状态下 空指令，不执行任何操作");
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
            if (doMagentOff())
            {
                updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            if (doMagentOff())
            { 
                log->info("终止指令执行，磁铁脱开，状态跳转: 手动");
                terminateCommand();
            }
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::doingWeldExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // log->info("碰钉--碰钉中 空指令，执行自动碰钉，完成碰钉后状态跳转: 退出--退出中");
            updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);  // 状态机离线调试(2024.09.03)
            if (doWeldAction(1))
            {
                updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            doWeldAction(-1);

            if (doMagentOff())
            {
                terminateCommand();
            }
            break;
        }
        case EExecutionCommand::eStopWeld:
        {
            log->info("停止碰钉，状态跳转: 碰钉--停止");
            doWeldAction(-1);
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eStopWeld);
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("暂停碰钉，状态跳转: 碰钉--待碰钉");
            doWeldAction(0);
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    } 
}

void CTask::stopWeldExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // log->info("碰钉--停止状态下 空指令，不做任何操作");
            break;
        }
        case EExecutionCommand::eMagentOff:
        {
            // log->info("磁铁脱开，状态跳转: 待吸合");
            if (doMagentOff())
            {
                updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            if (doMagentOff())
            {
                terminateCommand();
            }
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::quitingExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // log->info("退出--退出中状态下 空指令，移动到退出点，到达退出点后状态跳转: 手动");
            updateTopAndSubState(ETopState::eManual, ESubState::eReady); // 状态机离线调试(2023.09.03)
            m_Robot->setJointGroupMoveAbs(Postion_Home,JOINT_VEL_LIMIT);
            if(m_Robot->isJointReached(Postion_Home_qv))
            {
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
            log->info("停止运动，状态跳转: 退出--暂停");
            m_Robot->setLinkHalt();
            updateTopAndSubState(ETopState::eQuit, ESubState::ePause);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::pauseExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // log->info("退出-暂停状态下 空指令，不执行任何操作");
            break;
        }
        case EExecutionCommand::eQuit:
        {
            log->info("退出指令，状态跳转: 退出--退出中");
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CTask::terminateCommand()
{
    log->warn("停止运行，并进行状态跳转\n");
    
    TaskTerminate();

    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
}

void CTask::updateTopAndSubState(ETopState topState, ESubState subState)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_etopState = topState;
    m_esubState = subState;
}

void CTask::updateExecutionCommand(EExecutionCommand executionCommand)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_eexecutionCommand = executionCommand;
}

std::string CTask::getCurrentStateString()
{
    std::string stateString;
    auto itTopState = TopStateStringMap.find(m_etopState);
    stateString = itTopState->second;

    stateString += "--";

    auto itSubState = SubStateStringMap.find(m_esubState);
    stateString += itSubState->second;

    return stateString;
}

std::string CTask::getCurrentExecutionCommandString()
{
    auto it = ExecutionCommandStringMap.find(m_eexecutionCommand);

    return it->second;
}

void CTask::TranslateNumberToCMD()
{
    switch (m_manualOperator.TaskIndex)
    {
        case 0: // 空指令
        {
            updateExecutionCommand(EExecutionCommand::eNULL);
            break;
        }
        case 1: // 调平 
        {
            updateExecutionCommand(EExecutionCommand::eParallel);
            break;
        }
        case 4: // 对齐边线(定位)
        {
            updateExecutionCommand(EExecutionCommand::ePositioning);
            break;
        }
        case 16: // 吸合
        {
            updateExecutionCommand(EExecutionCommand::eMagentOn);
            break;
        }
        case 64: // 碰钉
        {
            updateExecutionCommand(EExecutionCommand::eAutoWeld);
            break;
        }
        case 2: // 脱开
        {
            updateExecutionCommand(EExecutionCommand::eMagentOff);
            break;
        }
        case 8: // 退出
        {
            updateExecutionCommand(EExecutionCommand::eQuit);
            break;
        }
        case 32: // 暂停
        {
            updateExecutionCommand(EExecutionCommand::ePause);
            break;
        }
        case 128: // 终止
        {
            updateExecutionCommand(EExecutionCommand::eTerminate);
            break;
        }
        default:
        {
            log->warn("invalid task index: {}", m_manualOperator.TaskIndex);
        }
    }
}

void CTask::TranslateManualTaskIndexNumberToCMD()
{
    switch (static_cast<stManualOperator::ETaskIndex>(m_manualOperator.TaskIndex))
    {
        case stManualOperator::ETaskIndex::None:
        {
            updateExecutionCommand(EExecutionCommand::eNULL);
            break;
        }
        case stManualOperator::ETaskIndex::Parallel:
        {
            updateExecutionCommand(EExecutionCommand::eParallel);
            break;
        }
        case stManualOperator::ETaskIndex::Positioning:
        {
            updateExecutionCommand(EExecutionCommand::ePositioning);
            break;
        }
        case stManualOperator::ETaskIndex::MagentOn:
        {
            updateExecutionCommand(EExecutionCommand::eMagentOn);
            break;
        }
        case stManualOperator::ETaskIndex::DoWeld:
        {
            updateExecutionCommand(EExecutionCommand::eAutoWeld);
            break;
        }
        case stManualOperator::ETaskIndex::MagentOff:
        {
            updateExecutionCommand(EExecutionCommand::eMagentOff);
            break;
        }
        case stManualOperator::ETaskIndex::Quit:
        {
            updateExecutionCommand(EExecutionCommand::eQuit);
            break;
        }
        case stManualOperator::ETaskIndex::Pause:
        {
            updateExecutionCommand(EExecutionCommand::ePause);
            break;
        }
        case stManualOperator::ETaskIndex::Terminate:
        {
            updateExecutionCommand(EExecutionCommand::eTerminate);
            break;
        }
    }
}