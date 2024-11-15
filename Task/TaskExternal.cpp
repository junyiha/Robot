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

    updateExecutionCommand();
}

void CTask::manualStateTransition()
{
    //1. 判断机器人是否处于就绪状态 
    bool robotReady = (m_LinkStatus.eLinkActState == eLINK_STANDSTILL);
    if (robotReady)
    {
        switch (m_eexecutionCommand)
        {
        case EExecutionCommand::eParallel:
        {
            updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eMagentOn:
        {
            updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
            break;
        }
        case EExecutionCommand::eQuit:
        {
            updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
            break;
        }
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
            break;
        }
        case EExecutionCommand::eParallel:
        {
            updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eMagentOn:
        {
            updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
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
            break;
        }
        case EExecutionCommand::eParallel:
        {
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
            UpdateLaserDistance();

            auto detectionResult = CheckParallelStateDecorator();
            switch (detectionResult)
            {
                case EDetectionInParallelResult::eDeviationIsLessThanThreshold:
                {
                    if (m_automatic_working_flag)
                        updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
                    else
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

void CTask::motionInParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            UpdateLaserDistance();

            auto result = CheckParallelStateDecorator();
            if (result == EDetectionInParallelResult::eNoWallDetected)
            {
                updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            }
            else if (result == EDetectionInParallelResult::eDeviationIsLessThanThreshold)
            { 
                m_Robot->setLinkHalt();
                updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            }
            else
            {
                // 计算偏差，控制机器人运动
                QVector<Eigen::Matrix4d> Dev_RT =  CMeasure::calPoseDeviation(m_stMeasuredata);
                QVector<double> tar_position = m_Robot->getTargetPose(Dev_RT[0]);

                double tar_pos[6] ;
                for(int i=0;i<6;i++)
                {
                    tar_pos[i] = tar_position[i];
                }

                log->info("{}: target_position: \n{}, {}, {}, {}, {}, {}", __LINE__,
                          tar_pos[0], tar_pos[1], tar_pos[2], tar_pos[3] * 57.3, tar_pos[4] * 57.3, tar_pos[5] * 57.3);
                auto temp_velocity = GP::End_Vel_Limit;
                temp_velocity.at(0) *= 7;
                temp_velocity.at(1) *= 7;
                temp_velocity.at(2) *= 7;
                m_Robot->setLinkMoveAbs(tar_pos, temp_velocity.data());
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
            break;
        }
        case EExecutionCommand::ePositioning:
        {
            updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eMagentOn:
        {
            updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            terminateCommand();
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

void CTask::detectionInPositioningExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            VisionResult vis_res = m_vision->getVisResult();
            if(!vis_res.status)
            { 
                break;
            }
            UpdateVisionResult(vis_res);

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
            UpdateLaserDistance();

            if (CheckParallelStateDecorator() == EDetectionInParallelResult::eNoWallDetected)
            {
                updateTopAndSubState(ETopState::eManual, ESubState::eReady);
                break;
            }
            static QVector<double> tar_position{0,0,0,0,0,0};
            static double tar_pos[6] = {0,0,0,0,0,0};
            if (!m_position_motion_flag) {
                QVector<Eigen::Matrix4d> Dev_RT = CMeasure::calPoseDeviation(m_stMeasuredata);
                tar_position = m_Robot->getTargetPose(Dev_RT[3]);  // 计算调整量

                for (int i = 0; i < 6; i++) {
                    tar_pos[i] = tar_position[i];
                }

                auto temp_velocity = GP::End_Vel_Limit;

                auto current_status = m_Robot->getLinkSta();
                if (std::fabs(current_status.stLinkActKin.LinkPos[0] - tar_pos[0]) < 8 || 
                    std::fabs(current_status.stLinkActKin.LinkPos[1] - tar_pos[1]) < 8)
                {
                    temp_velocity.at(0) *= 2;
                    temp_velocity.at(1) *= 2;
                    temp_velocity.at(2) *= 2;
                }
                else
                {
                    temp_velocity.at(0) *= 5;
                    temp_velocity.at(1) *= 5;
                    temp_velocity.at(2) *= 5;
                }

                log->info("{}: target_position: \n{}, {}, {}, {}, {}, {}", __LINE__,
                          tar_pos[0], tar_pos[1], tar_pos[2], tar_pos[3] * 57.3, tar_pos[4] * 57.3, tar_pos[5] * 57.3);
                m_Robot->setLinkMoveAbs(tar_pos, temp_velocity.data());
                stLinkStatus linkstatus = m_Robot->getLinkSta();
                m_position_motion_flag = true;
            }
            if (m_LinkStatus.eLinkActState == eLINK_STANDSTILL && 
                m_Robot->isEndReached(tar_position))
            {
                static int cnt = 0;
                if (cnt > 100) {
                    cnt = 0;
                    m_position_motion_flag = false;
                    updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
                }
                cnt++;
            }
            else
            {
                m_Robot->setLinkMoveAbs(tar_pos, GP::End_Vel_Limit.data());
            }

            break;
        }
        case EExecutionCommand::eTerminate:
        {
            m_position_motion_flag = false;
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            m_position_motion_flag = false;
            updateTopAndSubState(ETopState::ePositioning, ESubState::eReadyToPositioning);
            break;
        }
        default:
        {
            m_position_motion_flag = false;
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::readyToMagentOnExecutionCommand()
{
    if (m_automatic_working_flag)
        updateExecutionCommand(EExecutionCommand::eMagentOn);

    static int magent{ 0 };
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            if (magent == 1)
            {
                if (doMagentOn())
                {
                    magent = 0;
                    updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);
                }
            }
            break;
        }
        case EExecutionCommand::eParallel:
        {
            updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eMagentOn:
        {
            doMagentOn();
            magent = 1;
            break;
        }
        case EExecutionCommand::eQuit:
        {
            m_Comm->SetMagentAction(0,eMag_Off);
            updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            m_Comm->SetMagentAction(0,eMag_Off);
            terminateCommand();
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
    static int weld{0};
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
             if (weld == -1)
            {
                if (doMagentOff())
                {
                    weld = 0;
                    updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
                }
            }
            break;
        }
        case EExecutionCommand::eAutoWeld:
        {
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eDoingWeld);
            break;
        }
        case EExecutionCommand::eMagentOff:
        {
            doMagentOff();
            weld = -1;
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            m_Comm->SetMagentAction(0,eMag_Off);
            terminateCommand();
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
    static int weld{0};
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            if (weld == -1)
            {
                if (doMagentOff())
                {
                    weld = 0;
                    terminateCommand();
                }
                break;
            }
            if (NewDoWeldAction(1))
            {
                updateTopAndSubState(ETopState::eDoWeld, ESubState::eStopWeld);  // 碰钉结束，跳转到碰钉停止
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            NewDoWeldAction(-1);
            doMagentOff();
            weld = -1;
            break;
        }
        case EExecutionCommand::eStopWeld:
        {
            NewDoWeldAction(-1);
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eStopWeld);
            break;
        }
        case EExecutionCommand::ePause:
        {
            NewDoWeldAction(0);
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
    if (m_automatic_working_flag)
        updateExecutionCommand(EExecutionCommand::eMagentOff);

    static int weld{0};
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
           if (weld == -1)
            {
                if (doMagentOff())
                {
                    log->info("{} do magent off end...", __LINE__);
                    weld = 0;
                    if (m_automatic_working_flag)
                        updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
                    else
                        updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
                }
            }
            break;
        }
        case EExecutionCommand::eMagentOff:
        {
            doMagentOff();
            weld = -1;
            log->info("{} do magent off begin...", __LINE__);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            m_Comm->SetMagentAction(0,eMag_Off);
            terminateCommand();
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
            // 移动到退出位置
            std::vector<double> TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT(MAX_FREEDOM_LINK, 0.0);
            TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT = {1, 1, 1, 1, 0.3, 10, 5, 0.5, 3, 6};
            auto temp_value = GP::Position_Map[{GP::Working_Scenario, GP::PositionType::Quit}].value;

            // 整体联动无法停止，后续优化
            //m_Robot->setJointGroupMoveAbs(temp_value.data(), TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT.data());
            //QVector<double> value_qv(temp_value.begin(), temp_value.end());
            //if (m_Robot->isJointReached(value_qv))
            //{
            //    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            //}

            int tool_index{ 9 };
            int base_index{ 0 };
            m_Robot->setJointMoveAbs(tool_index, temp_value.at(tool_index), 4);
            m_Robot->setJointMoveAbs(base_index, temp_value.at(base_index), 2);
            if (std::abs(m_JointGroupStatus[tool_index].Position - temp_value.at(tool_index)) < 1 && 
                std::abs(m_JointGroupStatus[base_index].Position < temp_value.at(base_index)) < 1)
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CTask::terminateCommand()
{
    TaskTerminate();

    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
}

void CTask::UpdateLaserDistance()
{
    QVector<double> LaserDistance = m_Comm->getLasersDistance();

    m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
    m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
    m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
    m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];

//    // change laser position
//    std::vector<double> temp(std::begin(m_stMeasuredata.m_LaserDistance), std::end(m_stMeasuredata.m_LaserDistance));
//    m_stMeasuredata.m_LaserDistance[0] = temp[2];
//    m_stMeasuredata.m_LaserDistance[1] = temp[3];
//    m_stMeasuredata.m_LaserDistance[2] = temp[0];
//    m_stMeasuredata.m_LaserDistance[3] = temp[1];
}

void CTask::UpdateVisionResult(VisionResult& vis_res)
{
    std::copy(std::begin(vis_res.stData.m_LineDistance), std::end(vis_res.stData.m_LineDistance), m_stMeasuredata.m_LineDistance);
    std::copy(std::begin(vis_res.stData.m_bLineDistance), std::end(vis_res.stData.m_bLineDistance), m_stMeasuredata.m_bLineDistance);

    // change camera position
    std::vector<double> temp_vec(std::begin(m_stMeasuredata.m_LineDistance), std::end(m_stMeasuredata.m_LineDistance));
    std::vector<double> temp_flag_vec(std::begin(m_stMeasuredata.m_bLineDistance), std::end(m_stMeasuredata.m_bLineDistance));
    m_stMeasuredata.m_LineDistance[0] = temp_vec.at(3);
    m_stMeasuredata.m_LineDistance[1] = temp_vec.at(2);
    m_stMeasuredata.m_LineDistance[2] = temp_vec.at(1);
    m_stMeasuredata.m_LineDistance[3] = temp_vec.at(0);
    m_stMeasuredata.m_LineDistance[4] = temp_vec.at(5);
    m_stMeasuredata.m_LineDistance[5] = temp_vec.at(4);

    m_stMeasuredata.m_bLineDistance[0] = temp_flag_vec.at(3);
    m_stMeasuredata.m_bLineDistance[1] = temp_flag_vec.at(2);
    m_stMeasuredata.m_bLineDistance[2] = temp_flag_vec.at(1);
    m_stMeasuredata.m_bLineDistance[3] = temp_flag_vec.at(0);
    m_stMeasuredata.m_bLineDistance[4] = temp_flag_vec.at(5);
    m_stMeasuredata.m_bLineDistance[5] = temp_flag_vec.at(4);
}

void CTask::DoWelding(int tool_a, int tool_b, int key)
{
    std::bitset<8> value;
    switch (static_cast<ActionKey>(key))
    {
    case ActionKey::Grind_MovorOff1:
    {
        m_Comm->SetToolsAction(tool_a, eGrind_MovorOff);
        m_Comm->SetToolsAction(tool_b, eGrind_MovorOff);
        value = eGrind_MovorOff;
        break;
    }
    case ActionKey::Grind_OnorDown1:
    {
        m_Comm->SetToolsAction(tool_a, eGrind_OnorDown);
        m_Comm->SetToolsAction(tool_b, eGrind_OnorDown);
        value = eGrind_OnorDown;
        break;
    }
    case ActionKey::Grind_Up:
    {
        m_Comm->SetToolsAction(tool_a, eGrind_Up);
        m_Comm->SetToolsAction(tool_b, eGrind_Up);
        value = eGrind_Up;
        break;
    }
    case ActionKey::Grind_OnorDown2:
    {
        m_Comm->SetToolsAction(tool_a, eGrind_OnorDown);
        m_Comm->SetToolsAction(tool_b, eGrind_OnorDown);
        value = eGrind_OnorDown;
        break;
    }
    case ActionKey::Grind_MovorOff2:
    {
        m_Comm->SetToolsAction(tool_a, eGrind_MovorOff);
        m_Comm->SetToolsAction(tool_b, eGrind_MovorOff);
        value = eGrind_MovorOff;
        break;
    }
    case ActionKey::Weld_MovorDwon:
    {
        m_Comm->SetToolsAction(tool_a, eWeld_MovorDwon);
        m_Comm->SetToolsAction(tool_b, eWeld_MovorDwon);
        value = eWeld_MovorDwon;
        break;
    }
    case ActionKey::Weld_Fix:
    {
        m_Comm->SetToolsAction(tool_a, eWeld_Fix);
        m_Comm->SetToolsAction(tool_b, eWeld_Fix);
        value = eWeld_Fix;
        break;
    }
    case ActionKey::Weld_Up:
    {
        m_Comm->SetToolsAction(tool_a, eWeld_Up);
        m_Comm->SetToolsAction(tool_b, eWeld_Up);
        m_Comm->SetGunConnect(tool_a);
        m_Comm->SetGunConnect(tool_b);
        value = eWeld_Up;
        break;
    }
    case ActionKey::Weld_On:
    {
        m_Comm->SetToolsAction(tool_a, eWeld_On);
        m_Comm->SetToolsAction(tool_b, eWeld_On);
        value = eWeld_On;
        break;
    }
    case ActionKey::Weld_Down:
    {
        m_Comm->SetToolsAction(tool_a, eWeld_Down);
        m_Comm->SetToolsAction(tool_b, eWeld_Down);
        m_Comm->SetGunConnect(0);
        value = eWeld_Down;
        break;
    }
    case ActionKey::InitAction:
    {
        m_Comm->SetToolsAction(tool_a, eInitAction);
        m_Comm->SetToolsAction(tool_b, eInitAction);
        value = eInitAction;
        break;
    }
    }
    if (value.count() != 0)
    {
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        std::tm* now_tm = std::localtime(&timestamp);
        std::stringstream os;
        os << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");

        auto it = ValueMap.find(os.str());
        if (it != ValueMap.end())
        {
            it->second += value.count();
        }
        else
        {
            ValueMap[os.str()] = value.count();
        }
        log->warn("{}: value.count(): {}, timestamp: {}", __LINE__, value.count(), os.str());
    }
}


bool CTask::NewDoWeldAction(int execute)
{
    static quint8  index_tool = 1; //执行焊枪编号范围1~5
    static quint8  index_tool2 = 1; //执行焊枪编号范围1~5
    static quint8  index_act = 0;
    static int  time_cnt = 0; //周期计数，控制动作间隔
    static int  time_cnt2 = 0; //周期计数，控制动作间隔
    static bool offset_flag{ false };
    static bool end_flag{ true };

    //=====================结束碰钉 ==========================
    if (execute == -1)
    {
        unsigned char tem = ActionList[index_act] & 0b00100011;//关闭打磨、碰钉、定位气缸、打磨降
        tem |= 0b00100000; //碰钉枪下降
        log->info("{}: m_Comm->SetToolsAction({}, (E_WeldAction)tem); ", __LINE__, index_tool);
        log->info("{}: m_Comm->SetToolsAction({}, (E_WeldAction)tem); ", __LINE__, 11 - index_tool);
        if (offset_flag)
        {
            log->info("{}: m_Comm->SetToolsAction({}, (E_WeldAction)tem); ", __LINE__, index_tool2);
            log->info("{}: m_Comm->SetToolsAction({}, (E_WeldAction)tem); ", __LINE__, 11 - index_tool2);
        }
        log->info("{}: m_Comm->SetGunConnect(0);//关闭接触器 ", __LINE__);

        index_tool = 1;
        index_tool2 = 1;
        index_act = 0;
        time_cnt = 0;
        time_cnt2 = 0;
        log->info("结束碰钉作业");
        return true;
    }

    //=====================暂停碰钉 ==========================
    static quint32 pause_cnt = 0;
    if (execute == 0)
    {
        pause_cnt++;
        if (pause_cnt > 600)//暂停时间过长，停止
        {
            unsigned char tem = ActionList[index_act] & 0b10111111;//关闭打磨
            log->info("{}: m_Comm->SetToolsAction({}, (E_WeldAction)tem); ", __LINE__, index_tool);
            log->info("{}: m_Comm->SetToolsAction({}, (E_WeldAction)tem); ", __LINE__, 11 - index_tool);
            if (offset_flag)
            {
                log->info("{}: m_Comm->SetToolsAction({}, (E_WeldAction)tem); ", __LINE__, index_tool2);
                log->info("{}: m_Comm->SetToolsAction({}, (E_WeldAction)tem); ", __LINE__, 11 - index_tool2);
            }
            log->info("{}: m_Comm->SetGunConnect(0);//关闭接触器 ", __LINE__);

            pause_cnt = 0;
            log->warn("暂停时间过长，关闭打磨及接触器");
        }
        log->info("自动碰钉暂停");
        return true;
    }

    //=============== 执行10把焊枪轮次焊接1~5,6~10 ==============
    DoWelding(index_tool, 11 - index_tool, time_cnt);
    time_cnt++;
    if (time_cnt > static_cast<int>(ActionKey::End))
    {
        time_cnt = 0;
        index_tool = index_tool2 + 1;
    }

    if (!offset_flag)
    {
        if (time_cnt > 200)
        {
            index_tool2 = index_tool + 1;
            offset_flag = true;
        }
    }
    if (offset_flag)
    {
        if (end_flag)
        {
            DoWelding(index_tool2, 11 - index_tool2, time_cnt2);
            time_cnt2++;
            if (time_cnt2 > static_cast<int>(ActionKey::End))
            {
                time_cnt2 = 0;
                index_tool2 = index_tool + 1;
                if (index_tool2 > 5)
                {
                    end_flag = false;
                }
            }
        }
    }

    if (index_tool > 5)
    {
        return  true;
    }

    return false;
}

void CTask::updateTopAndSubState(ETopState topState, ESubState subState)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    log->info("状态转换: {}--{} ==> {}--{}", TopStateStringMap[m_etopState], SubStateStringMap[m_esubState], TopStateStringMap[topState], SubStateStringMap[subState]);

    m_etopState = topState;
    m_esubState = subState;
}

void CTask::updateExecutionCommand(EExecutionCommand executionCommand)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (executionCommand != EExecutionCommand::eNULL)
    {
        log->info("指令更新: {} ==> {}", ExecutionCommandStringMap[m_eexecutionCommand], ExecutionCommandStringMap[executionCommand]);
    }

    m_eexecutionCommand = executionCommand;
}

std::string CTask::getCurrentStateString() const
{
    std::string stateString;
    auto itTopState = TopStateStringMap.find(m_etopState);
    stateString = itTopState->second;

    stateString += "--";

    auto itSubState = SubStateStringMap.find(m_esubState);
    stateString += itSubState->second;

    return stateString;
}

std::string CTask::getCurrentExecutionCommandString() const
{
    auto it = ExecutionCommandStringMap.find(m_eexecutionCommand);

    return it->second;
}

bool CTask::checkSubState(ESubState subState) const
{
    return subState == m_esubState;
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

bool CTask::DoMagentOff()
{
    return doMagentOff();
}

bool CTask::DoWeldAction(int index)
{
    return NewDoWeldAction(index);
}

bool CTask::GetWorkingMode()
{
    return m_automatic_working_flag;
}

void CTask::SetWorkdingMode(const bool mode)
{
    m_automatic_working_flag = mode;
}
