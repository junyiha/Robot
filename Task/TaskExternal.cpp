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
        log->warn("第二层状态数据无效!!!");
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        log->info("{}: 暂停指令不执行任何操作", getCurrentStateString());
        break;
    }
    default:
    {
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::detectionInParallelExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        log->info("{} 调用函数，根据激光数值判断壁面状态", __LINE__);
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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

            // log->info("{} tar_pos:{},{},{},{},{},{}", __LINE__, tar_pos[0], tar_pos[1], tar_pos[2],
            //           tar_pos[3] * 57.3, tar_pos[4] * 57.3, tar_pos[5] * 57.3);

            // stLinkStatus linkstatus = m_Robot->getLinkSta();
            // log->info("m_Robot->setLinkMoveAbs(tar_pos_differ:{},{},{},{},{},{}",
            //           tar_pos[0] - linkstatus.stLinkActKin.LinkPos[0], tar_pos[1] - linkstatus.stLinkActKin.LinkPos[1],
            //           tar_pos[2] - linkstatus.stLinkActKin.LinkPos[2], tar_pos[3] * 57.3 - linkstatus.stLinkActKin.LinkPos[3] * 57.3,
            //           tar_pos[4] * 57.3 - linkstatus.stLinkActKin.LinkPos[4] * 57.3, tar_pos[5] * 57.3 - linkstatus.stLinkActKin.LinkPos[5] * 57.3);
            m_Robot->setLinkMoveAbs(tar_pos, GP::End_Vel_Limit.data());
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        log->info("{}: 暂停指令不执行任何操作", getCurrentStateString());
        break;
    }
    default:
    {
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        if (!vis_res.lineStatus)
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
        log->info("{}: 暂停指令不执行任何操作", getCurrentStateString());
        break;
    }
    default:
    {
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
            log->warn("{} 反馈异常，末端激光偏差过大，状态跳转: 手动--准备", __LINE__);
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            break;
        }

        static QVector<double> tar_position{ 0, 0, 0, 0, 0, 0 };
        static double tar_pos[6] = { 0, 0, 0, 0, 0, 0 };
        if (!m_position_motion_flag)
        {
            QVector<Eigen::Matrix4d> Dev_RT = CMeasure::calPoseDeviation(m_stMeasuredata);
            tar_position = m_Robot->getTargetPose(Dev_RT[3]); // 计算调整量

            for (int i = 0; i < 6; i++)
            {
                tar_pos[i] = tar_position[i];
            }

            m_Robot->setLinkMoveAbs(tar_pos, GP::End_Vel_Position.data());
            stLinkStatus linkstatus = m_Robot->getLinkSta();
            log->info("{} tar_pos:{}, {}, {}, {}, {}, {}\ncurrent_pos: {}, {}, {}, {}, {}, {}", __LINE__,
                      tar_pos[0], tar_pos[1], tar_pos[2], tar_pos[3] * 57.3, tar_pos[4] * 57.3, tar_pos[5] * 57.3,
                      linkstatus.stLinkActKin.LinkPos[0], linkstatus.stLinkActKin.LinkPos[1], linkstatus.stLinkActKin.LinkPos[2],
                      linkstatus.stLinkActKin.LinkPos[3] * 57.3, linkstatus.stLinkActKin.LinkPos[4] * 57.3, linkstatus.stLinkActKin.LinkPos[5] * 57.3);
            log->info("{} tar_pos - current_pos:{},{},{},{},{},{}", __LINE__,
                      tar_pos[0] - linkstatus.stLinkActKin.LinkPos[0], tar_pos[1] - linkstatus.stLinkActKin.LinkPos[1],
                      tar_pos[2] - linkstatus.stLinkActKin.LinkPos[2], tar_pos[3] * 57.3 - linkstatus.stLinkActKin.LinkPos[3] * 57.3,
                      tar_pos[4] * 57.3 - linkstatus.stLinkActKin.LinkPos[4] * 57.3, tar_pos[5] * 57.3 - linkstatus.stLinkActKin.LinkPos[5] * 57.3);
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
                log->warn("{} 定位运动完成，状态跳转: 定位--检测", __LINE__);
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
            log->warn("{} 反馈异常，末端激光偏差过大，状态跳转: 手动--准备", __LINE__);
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
        if (!vis_res.lineStatus)
            break;
        UpdateVisionResult(vis_res);

        result = CheckSidelineStateDecorator();
#endif
        switch (result)
        {
        case EDetectionInPositioningResult::eDeviationIsLessThanThreshold:
        {
            CalculatedAdjustmentOfLift();
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eLiftMotion);
            break;
        }
        case EDetectionInPositioningResult::eEndAdjustmentDataIsValid:
        {
            CalculatedAdjustmentOfSideline();
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eSidelineMotion);
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::sidelineMotionInFitBoardExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        m_Robot->setLinkMoveAbs(m_fit_board_target_pose.data(), GP::End_Vel_Position.data());

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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
            double velocity = LINK_0_JOINT_MAX_VEL[GP::TOOL_LIFTING] * 0.5;
            m_Robot->setJointMoveAbs(GP::TOOL_LIFTING, m_lift_tool, velocity);
            log->info("{} joint: {}, target position: {}, velocity: {}", __LINE__, GP::TOOL_LIFTING, m_lift_tool, velocity);
            lift_motion_flag = true;
            break;
        }

        if (lift_motion_flag)
        {
            // 判断是否运动结束
            auto status = m_Robot->getJointGroupSta();
            bool flag_state = status[GP::TOOL_LIFTING].eState == eAxis_STANDSTILL;
            bool flag_motion = std::fabs(status[GP::TOOL_LIFTING].Position - m_lift_tool) < 1;
            log->info("{} state: {}, position: {}", __LINE__, static_cast<int>(status[GP::TOOL_LIFTING].eState), status[GP::TOOL_LIFTING].Position);
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        log->info("{}: 暂停指令不执行任何操作", getCurrentStateString());
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        updateTopAndSubState(ETopState::eManual, ESubState::eReady);
        break;
    }
    default:
    {
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
    log->info("{}: motion index: {}", __LINE__, m_motion_index);
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
            log->info("{}: 贴合调整目标距离为：{}", __LINE__, BOARDING_MOTION_QUE[i]);
            break;
        }
    }

    // 计算调整量
    double average_distance = std::accumulate(laserDistance.begin(), laserDistance.end(), 0.0) / laserDistance.size();
    m_lift_tool = average_distance - BOARDING_MOTION_QUE.at(motion_index);
    m_lift_tool += m_JointGroupStatus[GP::TOOL_LIFTING].Position;
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

    log->info("状态跳转: {}--{} ==> {}--{}", TopStateStringMap[m_etopState].first, SubStateStringMap[m_esubState].first,
              TopStateStringMap[topState].first, SubStateStringMap[subState].first);

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
        log->error("{} invalid task index: {}", __LINE__, m_manualOperator.TaskIndex);
    }
    }
}