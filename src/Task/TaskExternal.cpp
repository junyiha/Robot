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
    case ETopState::eFitBoard:
    {
        fitBoardStateTransition();
        break;
    }
    case ETopState::eQuit:
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
        SPDLOG_ERROR("第二层状态数据无效!!!");
    }
    }
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
        SPDLOG_ERROR("第二层状态数据无效!!!");
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
        SPDLOG_ERROR("第二层状态数据无效!!!");
    }
    }
}

void CTask::fitBoardStateTransition()
{
    switch (m_esubState)
    {
    case ESubState::eReadyToFitBoard:
    {
        readyToFitBoardExecutionCommand();
        break;
    }
    case ESubState::eDetection:
    {
        detectionInFitBoardExecutionCommand();
        break;
    }
    case ESubState::eSidelineMotion:
    {
        sidelineMotionInFitBoardExecutionCommand();
        break;
    }
    case ESubState::eLiftMotion:
    {
        liftMotionInFitBoardExecutionCommand();
        break;
    }
    case ESubState::eFitBoardFinished:
    {
        fitBoardFinishedExecutionCommand();
        break;
    }
    default:
    {
        SPDLOG_ERROR("第二层状态数据无效!!!");
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
        SPDLOG_ERROR("第二层状态数据无效!!!");
    }
    }
}

/////////////////////////////////////////////--指令判断--/////////////////////////////////////////////////

void CTask::readyExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        break;
    }
    case EExecutionCommand::ePause:
    {
        m_Robot->setLinkHalt();
        return;
    }
    case EExecutionCommand::eTerminate:
    {
        m_Robot->setLinkHalt();
        return;
    }
    case EExecutionCommand::eParallel:
    {
        updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
        return;
    }
    case EExecutionCommand::eFitBoard:
    {
        updateTopAndSubState(ETopState::eFitBoard, ESubState::eDetection);
        return;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }

    Manual();
}

void CTask::notReadyExecutionCommand()
{
    bool robotReady;
#ifdef TEST_TASK_STATEMACHINE_
    robotReady = true;
#else
    robotReady = (m_LinkStatus.eLinkActState == eLINK_STANDSTILL);
#endif
    if (robotReady)
        updateTopAndSubState(ETopState::eManual, ESubState::eReady);

    Manual();
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
        SPDLOG_INFO("{}: 暂停指令不执行任何操作", getCurrentStateString());
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::detectionInParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        SPDLOG_INFO("调用函数，根据激光数值判断壁面状态");
        EDetectionInParallelResult detectionResult;
#ifdef TEST_TASK_STATEMACHINE_
        detectionResult = EDetectionInParallelResult::eDeviationIsLessThanThreshold;
#else
        UpdateLaserDistance();
        detectionResult = CheckParallelStateDecorator();
#endif
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
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        switch (result)
        {
        case EDetectionInParallelResult::eNoWallDetected:
        {
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            break;
        }
        case EDetectionInParallelResult::eDeviationIsLessThanThreshold:
        {
            m_Robot->setLinkHalt();
            updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            break;
        }
        case EDetectionInParallelResult::eDistanceMeetsRequirement:
        {
            // 计算偏差，控制机器人运动
            QVector<Eigen::Matrix4d> Dev_RT = CMeasure::calPoseDeviation(m_stMeasuredata);
            QVector<double> tar_position = m_Robot->getTargetPose(Dev_RT[0]);
            double* tar_pos = tar_position.data();

            std::vector<double> parallel_velocity = GP::End_Vel_Limit;
            int scalar{ 1 };
            scalar = GP::Working_Scenario == GP::WorkingScenario::Top ? 7 : 3;
            parallel_velocity.at(0) = scalar;
            parallel_velocity.at(1) = scalar;
            parallel_velocity.at(2) = scalar;

            m_Robot->setLinkMoveAbs(tar_pos, parallel_velocity.data());
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
        m_Robot->setLinkHalt();
        updateTopAndSubState(ETopState::eParallel, ESubState::eReadyToParallel);
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
    case EExecutionCommand::eTerminate:
    {
        terminateCommand();
        break;
    }
    case EExecutionCommand::ePause:
    {
        SPDLOG_INFO("{}: 暂停指令不执行任何操作", getCurrentStateString());
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::detectionInPositioningExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        EDetectionInPositioningResult detectResult;
#ifdef TEST_TASK_STATEMACHINE_
        detectResult = EDetectionInPositioningResult::eDeviationIsLessThanThreshold;
#else
        VisionResult vis_res = m_vision->getVisResult();
        if (!vis_res.lineStatus && !vis_res.laserStatus)
            break;

        UpdateVisionResult(vis_res);
        detectResult = CheckSidelineStateDecorator();
#endif
        switch (detectResult)
        {
        case EDetectionInPositioningResult::eDeviationIsLessThanThreshold:
        {
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eReadyToFitBoard);
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
        SPDLOG_INFO("{}: 暂停指令不执行任何操作", getCurrentStateString());
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::motionInPositioningExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        UpdateLaserDistance();
        if (CheckParallelStateDecorator() == EDetectionInParallelResult::eNoWallDetected)
        {
            SPDLOG_WARN("反馈异常，末端激光偏差过大，状态跳转: 手动--准备");
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            break;
        }

        static QVector<double> tar_position{ 0, 0, 0, 0, 0, 0 };
        static double tar_pos[6] = { 0, 0, 0, 0, 0, 0 };
        static std::vector<double> temp_vel = GP::End_Vel_Limit;
        if (!m_position_motion_flag)
        {
            QVector<Eigen::Matrix4d> Dev_RT = CMeasure::calPoseDeviation(m_stMeasuredata);
            tar_position = m_Robot->getTargetPose(Dev_RT[3]); // 计算调整量

            for (int i = 0; i < 6; i++)
            {
                tar_pos[i] = tar_position[i];
            }

            if (std::fabs(m_LinkStatus.stLinkActKin.LinkPos[0] - tar_pos[0]) < 5 ||
                std::fabs(m_LinkStatus.stLinkActKin.LinkPos[1] - tar_pos[1]) < 5)
            {
                temp_vel.at(0) *= 0.5;
                temp_vel.at(1) *= 0.5;
                temp_vel.at(2) *= 0.5;
            }
            else
            {
                temp_vel.at(0) *= 2;
                temp_vel.at(1) *= 2;
                temp_vel.at(2) *= 2;
            }

            m_Robot->setLinkMoveAbs(tar_pos, temp_vel.data());
            m_position_motion_flag = true;
        }
        if (m_LinkStatus.eLinkActState == eLINK_STANDSTILL &&
            m_Robot->isEndReached(tar_position))
        {
            static int cnt = 0;
            if (cnt > 100)
            {
                cnt = 0;
                m_position_motion_flag = false;
                SPDLOG_WARN("{} 定位运动完成，状态跳转: 定位--检测");
                updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
            }
            cnt++;
        }
        else
        {
            temp_vel.at(0) = 0.5;
            temp_vel.at(1) = 0.5;
            temp_vel.at(2) = 0.5;
            m_Robot->setLinkMoveAbs(tar_pos, temp_vel.data());
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
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::readyToFitBoardExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        break;
    }
    case EExecutionCommand::eFitBoard:
    {
        updateTopAndSubState(ETopState::eFitBoard, ESubState::eDetection);
        break;
    }
    case EExecutionCommand::ePositioning:
    {
        updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
        break;
    }
    case EExecutionCommand::ePause:
    {
        m_Robot->setLinkHalt();
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        updateTopAndSubState(ETopState::eManual, ESubState::eReady);
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::detectionInFitBoardExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        EDetectionInPositioningResult result;
#ifdef TEST_TASK_STATEMACHINE_
        result = EDetectionInPositioningResult::eDeviationIsLessThanThreshold;
#else
        UpdateLaserDistance();
        if (CheckParallelStateDecorator() == EDetectionInParallelResult::eNoWallDetected)
        {
            SPDLOG_WARN("反馈异常，末端激光偏差过大，状态跳转: 手动--准备");
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            break;
        }
        // 判断板壁距离
        if (CheckFitBoardState() == EDetectionInFitBoardResult::eDeviationIsLessThanThreshold)
        {
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eFitBoardFinished);
            break;
        }

        VisionResult vis_res = m_vision->getVisResult();
        if (!vis_res.lineStatus && !vis_res.laserStatus)
            break;
        UpdateVisionResult(vis_res);

        result = CheckSidelineStateDecorator();
#endif
        static int cnt{ 0 };
        switch (result)
        {
        case EDetectionInPositioningResult::eDeviationIsLessThanThreshold:
        {
            CalculatedAdjustmentOfLift();
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eLiftMotion);
            cnt = 0;
            break;
        }
        case EDetectionInPositioningResult::eEndAdjustmentDataIsValid:
        {
            if (cnt < 40) // 1000ms / 50ms
            {
                cnt++;
                SPDLOG_INFO("count: {}", cnt);
                break;
            }
            CalculatedAdjustmentOfSideline();
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eSidelineMotion);
            cnt = 0;
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
    case EExecutionCommand::ePause:
    {
        updateTopAndSubState(ETopState::eFitBoard, ESubState::eReadyToFitBoard);
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        updateTopAndSubState(ETopState::eManual, ESubState::eReady);
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::sidelineMotionInFitBoardExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        auto temp_vel = GP::End_Vel_Limit;
        temp_vel.at(0) = 0.5;
        temp_vel.at(1) = 0.5;
        temp_vel.at(2) = 0.5;
        m_Robot->setLinkMoveAbs(m_fit_board_target_pose.data(), temp_vel.data());

        QVector<double> tar_position(m_fit_board_target_pose.begin(), m_fit_board_target_pose.end());
        if (m_LinkStatus.eLinkActState == eLINK_STANDSTILL &&
            m_Robot->isEndReached(tar_position))
        {
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eDetection);
        }
        break;
    }
    case EExecutionCommand::ePause:
    {
        updateTopAndSubState(ETopState::eFitBoard, ESubState::eReadyToFitBoard);
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        updateTopAndSubState(ETopState::eManual, ESubState::eReady);
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::liftMotionInFitBoardExecutionCommand()
{
    static bool lift_motion_flag = false;
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        if (!lift_motion_flag)
        {
            double scaler{ 1 };
            if (m_motion_index > 3)
            {
                scaler = 0.5;
            }
            double velocity = RobotConfigMap.at(GP::TOOL_LIFTING).max_velocity * scaler;
            m_Robot->setJointMoveAbs(GP::TOOL_LIFTING, m_lift_tool, velocity);
            SPDLOG_INFO("joint: {}, target position: {}, velocity: {}", GP::TOOL_LIFTING, m_lift_tool, velocity);
            lift_motion_flag = true;
            break;
        }

        if (lift_motion_flag)
        {
            // 判断是否运动结束
            auto status = m_Robot->getJointGroupSta();
            bool flag_state = status[GP::TOOL_LIFTING].eState == eAxis_STANDSTILL;
            bool flag_motion = std::fabs(status[GP::TOOL_LIFTING].Position - m_lift_tool) < 1;
            SPDLOG_INFO("state: {}, position: {}", static_cast<int>(status[GP::TOOL_LIFTING].eState), status[GP::TOOL_LIFTING].Position);
            if (flag_motion && flag_state)
            {
                updateTopAndSubState(ETopState::eFitBoard, ESubState::eDetection);
                lift_motion_flag = false;
            }
            else if (!flag_motion && flag_state)
            {
                lift_motion_flag = false;
            }
        }

        break;
    }
    case EExecutionCommand::ePause:
    {
        if (lift_motion_flag)
        {
            m_Robot->setRobotHalt();
            lift_motion_flag = false;
        }
        updateTopAndSubState(ETopState::eFitBoard, ESubState::eReadyToFitBoard);
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        if (lift_motion_flag)
        {
            m_Robot->setRobotHalt();
            lift_motion_flag = false;
        }
        updateTopAndSubState(ETopState::eManual, ESubState::eReady);
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::fitBoardFinishedExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        break;
    }
    case EExecutionCommand::eFitBoard:
    {
        updateTopAndSubState(ETopState::eFitBoard, ESubState::eDetection);
        break;
    }
    case EExecutionCommand::eQuit:
    {
        updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
        break;
    }
    case EExecutionCommand::ePause:
    {
        SPDLOG_INFO("{}: 暂停指令不执行任何操作", getCurrentStateString());
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        updateTopAndSubState(ETopState::eManual, ESubState::eReady);
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::quitingExecutionCommand()
{
    static int cnt{ 0 };
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        // 推缸退出
        m_Comm->SetCylinder(-1);
        if (cnt > 200) // 10s
        {
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            cnt = 0;
            m_Comm->SetCylinder(0);
        }
        cnt++;
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        cnt = 0;
        m_Comm->SetCylinder(0);
        terminateCommand();
        break;
    }
    case EExecutionCommand::ePause:
    {
        cnt = 0;
        m_Comm->SetCylinder(0);
        m_Robot->setLinkHalt();
        updateTopAndSubState(ETopState::eQuit, ESubState::ePause);
        break;
    }
    default:
    {
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        SPDLOG_WARN("{} 执行指令不合法!指令: {}", getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CTask::terminateCommand()
{
    TaskTerminate();

    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
}

void CTask::CalculatedAdjustmentOfSideline()
{
    // 计算调整量
    std::vector<double> laserDistance(std::begin(m_stMeasuredata.m_LaserDistance), std::end(m_stMeasuredata.m_LaserDistance));
    double average_distance = std::accumulate(laserDistance.begin(), laserDistance.end(), 0.0) / laserDistance.size();
    QVector<Eigen::Matrix4d> Dev_RT = CMeasure::calPoseDeviation(m_stMeasuredata, average_distance);
    QVector<double> tar_position{ 0, 0, 0, 0, 0, 0 };
    tar_position = m_Robot->getTargetPose(Dev_RT[5]); // 计算调整量
    m_fit_board_target_pose = std::vector(tar_position.begin(), tar_position.end());

    stLinkStatus linkstatus = m_Robot->getLinkSta();
    SPDLOG_INFO("current_pos: {}, {}, {}, {}, {}, {}", linkstatus.stLinkActKin.LinkPos[0], linkstatus.stLinkActKin.LinkPos[1], linkstatus.stLinkActKin.LinkPos[2],
                      linkstatus.stLinkActKin.LinkPos[3] * 57.3, linkstatus.stLinkActKin.LinkPos[4] * 57.3, linkstatus.stLinkActKin.LinkPos[5] * 57.3);
    SPDLOG_INFO("m_fit_board_target_pose: {},{},{},{},{},{}", m_fit_board_target_pose.at(0),
                                                              m_fit_board_target_pose.at(1),
                                                              m_fit_board_target_pose.at(2),
                                                              m_fit_board_target_pose.at(3),
                                                              m_fit_board_target_pose.at(4),
                                                              m_fit_board_target_pose.at(5));
}

void CTask::CalculatedAdjustmentOfLift()
{
    auto motion_index = BOARDING_MOTION_QUE.size();

    // 计算运动索引
    std::vector<double> laserDistance(std::begin(m_stMeasuredata.m_LaserDistance), std::end(m_stMeasuredata.m_LaserDistance));
    auto minDistance = std::min_element(laserDistance.begin(), laserDistance.end());
    for (int i = 0; i < BOARDING_MOTION_QUE.size(); ++i)
    {
        if (BOARDING_MOTION_QUE[i] + 2 < *minDistance)
        {
            motion_index = i;
            SPDLOG_INFO("贴合调整目标距离为：{}", BOARDING_MOTION_QUE[i]);
            break;
        }
    }

    // 计算调整量
    double average_distance = std::accumulate(laserDistance.begin(), laserDistance.end(), 0.0) / laserDistance.size();
    m_lift_tool = average_distance - BOARDING_MOTION_QUE.at(motion_index);
    m_lift_tool += m_JointGroupStatus[GP::TOOL_LIFTING].Position;
    m_motion_index = motion_index;
}

void CTask::UpdateLaserDistance()
{
    // 取雷达数据
    QVector<double> LaserDistance = m_Comm->getLasersDistanceBoardingByBojke();

    m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
    m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
    m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
    m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];
}



void CTask::UpdateVisionResult(VisionResult& vis_res)
{
    double sum{ 0.0 };
    for (int i = 0; i < 4; i++)
        sum += m_stMeasuredata.m_LaserDistance[i];

    double laser_average = sum / 4;

    double k = (laser_average + 460) / 470;

    for (int i = 0; i < 6; i++)
        vis_res.stData.m_LineDistance[i] *= k;

    int laser2camera[4] = {
            4, 5, 1, 3
    };  // 轮廓激光映射到对应相机


    //1.0 轮廓激光结果校验视觉结果
    for (int i = 0; i < 4; i++)
    {
        if (vis_res.stData.m_bLaserProfile[i]) // 如果轮廓激光数据有效
        {
            double laserTemp = vis_res.stData.m_LaserGapDistance[i] - 15;
            if (vis_res.stData.m_bLineDistance[laser2camera[i]]) // 视觉结果有效
            {
                if (abs(vis_res.stData.m_LineDistance[laser2camera[i]] - laserTemp) > 5)  // 轮廓结果与对应视觉的偏差小于5mm认为视觉数据是有效的
                {
                    vis_res.stData.m_bLineDistance[laser2camera[i]] = false;
                }
            }
        }
    }

    //2.0 检查视觉数据有效性
    bool visLong = (vis_res.stData.m_bLineDistance[0] && vis_res.stData.m_bLineDistance[2]) ||
        (vis_res.stData.m_bLineDistance[1] && vis_res.stData.m_bLineDistance[3]);

    bool visShort = vis_res.stData.m_bLineDistance[4] || vis_res.stData.m_bLineDistance[5];
    bool visDataIsValid = visLong && visShort;

    //3.0 检查轮廓激光数据有效性
    bool laserLong = vis_res.stData.m_bLaserProfile[2] && vis_res.stData.m_bLaserProfile[3];
    bool laserShort = vis_res.stData.m_bLaserProfile[0] || vis_res.stData.m_bLaserProfile[1];
    bool laserDataValid = laserLong && laserShort;
    laserDataValid = false;

    // 1. 提高优先级
    //4.0 融合策略
    if (visDataIsValid)
    {
        // 视觉无需做任何处理
    }
    else if (laserDataValid)
    {
        // 清除原有视觉数据
        for (int i = 0; i < 6; i++)
        {
            vis_res.stData.m_bLineDistance[i] = false;
            vis_res.stData.m_LineDistance[i] = 0.0;
        }
        // 轮廓激光重新赋值
        for (int i = 0; i < 4; i++)
        {
            vis_res.stData.m_bLineDistance[laser2camera[i]] = true;
            vis_res.stData.m_LineDistance[laser2camera[i]] = vis_res.stData.m_LaserGapDistance[i] - 15;
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            // if(i==1){continue;}
            if (vis_res.stData.m_bLaserProfile[i]) // 如果轮廓激光数据有效
            {
                double laserTemp = vis_res.stData.m_LaserGapDistance[i] - 15;
                if (vis_res.stData.m_bLineDistance[laser2camera[i]]) // 视觉结果有效
                {   // 轮廓激光对应的相机位置有效
                    if (abs(vis_res.stData.m_LineDistance[laser2camera[i]] - laserTemp) < 5)
                    {
                        // vis_res.stData.m_bLineDistance[laser2camera[i]] = true;
                        SPDLOG_INFO("vis_res.stData.m_LineDistance[laser2camera[i]]: {}\nlaserTemp: {}", vis_res.stData.m_LineDistance[laser2camera[i]], laserTemp);
                        // vis_res.stData.m_LineDistance[laser2camera[i]] = std::min(vis_res.stData.m_LineDistance[laser2camera[i]],laserTemp);
                        // vis_res.stData.m_LineDistance[laser2camera[i]] = vis_res.stData.m_LineDistance[laser2camera[i]];
                    }
                    else
                    {
                        // 有待于进一步观察
                        if (laserTemp < 70)
                        { //限制轮廓激光结果值不要太大
            // vis_res.stData.m_bLineDistance[laser2camera[i]] = true;
                            vis_res.stData.m_LineDistance[laser2camera[i]] = laserTemp;
                        }
                    }

                }
                else
                { // 相机测量数据无效

                    if (laserTemp < 70)
                    { //限制轮廓激光结果值不要太大
                        vis_res.stData.m_bLineDistance[laser2camera[i]] = true;
                        vis_res.stData.m_LineDistance[laser2camera[i]] = laserTemp;
                    }

                }
            }

        }
    }

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

void CTask::updateTopAndSubState(ETopState topState, ESubState subState)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    SPDLOG_INFO("状态跳转: {}--{} ==> {}--{}", TopStateStringMap[m_etopState].first, SubStateStringMap[m_esubState].first,
              TopStateStringMap[topState].first, SubStateStringMap[subState].first);

    m_etopState = topState;
    m_esubState = subState;
}

void CTask::updateExecutionCommand(EExecutionCommand executionCommand)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (executionCommand != EExecutionCommand::eNULL)
    {
        SPDLOG_INFO("指令更新: {} ==> {}", ExecutionCommandStringMap[m_eexecutionCommand], ExecutionCommandStringMap[executionCommand]);
    }

    if (executionCommand == EExecutionCommand::eTerminate)
    {
        m_single_job_flag.store(false);
    }

    m_eexecutionCommand = executionCommand;
}

std::string CTask::getCurrentStateString() const
{
    std::string stateString;
    auto itTopState = TopStateStringMap.find(m_etopState);
    stateString = itTopState->second.first;

    stateString += "--";

    auto itSubState = SubStateStringMap.find(m_esubState);
    stateString += itSubState->second.first;

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

bool CTask::SingleSideLine()
{
    int cnt{ 0 };
    m_single_job_flag.store(true);
    while (true)
    {
        if (!m_single_job_flag.load())
        {
            SPDLOG_WARN("终止指令,退出单次对边任务!!!");
            m_Robot->setRobotHalt();
            return false;
        }
        if (cnt > 1000)
        {
            SPDLOG_ERROR("侧线调整超时");
            return false;
        }

        UpdateLaserDistance();
        if (CheckParallelStateDecorator() == EDetectionInParallelResult::eNoWallDetected)
        {
            SPDLOG_ERROR("未检测到墙壁或者墙壁距离过远");
            return false;
        }
        if (CheckFitBoardState() == EDetectionInFitBoardResult::eDeviationIsLessThanThreshold)
        {
            SPDLOG_INFO("板壁距离检测完成");
            return true;
        }
        VisionResult vis_res = m_vision->getVisResult();
        if (!vis_res.lineStatus && !vis_res.laserStatus)
        {
            SPDLOG_ERROR("视觉数据无效");
            return false;
        }
        UpdateVisionResult(vis_res);

        auto result = CheckSidelineStateDecorator();
        if (result != EDetectionInPositioningResult::eEndAdjustmentDataIsValid)
        {
            SPDLOG_ERROR("未检测到有效数据");
            return false;
        }

        CalculatedAdjustmentOfSideline();
        auto temp_vel = GP::End_Vel_Limit;
        temp_vel.at(0) = 0.5;
        temp_vel.at(1) = 0.5;
        temp_vel.at(2) = 0.5;
        m_Robot->setLinkMoveAbs(m_fit_board_target_pose.data(), temp_vel.data());

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        cnt++;
    }
}

bool CTask::SingleParallel()
{
    m_single_job_flag.store(true);
    while (true)
    {
        UpdateLaserDistance();
        if (!m_single_job_flag.load())
        {
            SPDLOG_WARN("终止指令,退出单次调平任务!!!");
            m_Robot->setRobotHalt();
            return false;
        }
        auto result = CheckParallelStateDecorator();
        if (result == EDetectionInParallelResult::eDeviationIsLessThanThreshold)
        {
            SPDLOG_INFO("平行度检测完成");
            return true;
        }
        if (result == EDetectionInParallelResult::eNoWallDetected)
        {
            SPDLOG_ERROR("未检测到墙壁或者墙壁距离过远");
            return false;
        }
        // 计算偏差，控制机器人运动
        QVector<Eigen::Matrix4d> Dev_RT = CMeasure::calPoseDeviation(m_stMeasuredata);
        QVector<double> tar_position = m_Robot->getTargetPose(Dev_RT[0]);
        double* tar_pos = tar_position.data();

        std::vector<double> parallel_velocity = GP::End_Vel_Limit;

        int cnt{ 0 };
        m_Robot->setLinkMoveAbs(tar_pos, parallel_velocity.data());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void CTask::SecondPush()
{
    if (GP::Working_Scenario != GP::WorkingScenario::Top)
    {
        SPDLOG_WARN("非顶板作业场景，二次举升不执行任何动作");
        return;
    }

    UpdateLaserDistance();

    std::vector<double> laser_distance(std::begin(m_stMeasuredata.m_LaserDistance), std::end(m_stMeasuredata.m_LaserDistance));
    auto max_distance = std::max_element(laser_distance.begin(), laser_distance.end());
    if (*max_distance < 120 || *max_distance > 800)
    {
        SPDLOG_WARN("激光距离过小或过大，无需二次举升");
        return;
    }

    double offset = *max_distance - GP::Second_Push_Distance;

    std::vector<double> cur_pos(m_LinkStatus.stLinkActKin.LinkPos, m_LinkStatus.stLinkActKin.LinkPos + 6);
    cur_pos.at(2) += offset;

    m_Robot->setLinkMoveAbs(cur_pos.data(), GP::End_Vel_Limit.data());
}

void CTask::SecondQuit()
{
    if (GP::Working_Scenario == GP::WorkingScenario::Top)
    {
        m_Robot->setJointMoveAbs(9, 875.0, RobotConfigMap.at(9).max_velocity);  // 工具升降
        return;
    }

    std::vector<double> cur_pos(m_LinkStatus.stLinkActKin.LinkPos, m_LinkStatus.stLinkActKin.LinkPos + 6);
    cur_pos.at(2) -= GP::Second_Quit_Distance;

    m_Robot->setLinkMoveAbs(cur_pos.data(), GP::End_Vel_Limit.data());
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
    case stManualOperator::ETaskIndex::FitBoard:
    {
        updateExecutionCommand(EExecutionCommand::eFitBoard);
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
    default:
    {
        SPDLOG_ERROR("invalid task index: {}", m_manualOperator.TaskIndex);
    }
    }
}