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
        case ESubState::eMotion:
        {
            motionInFitBoardExecutionCommand();
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
    log->warn("机器人未就绪，离线状态！！！");

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
        detectionResult = CheckParallelStateDecorator(m_stMeasuredata.m_LaserDistance);
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
    static QVector<double> last_tar_position{ 0, 0, 0, 0, 0, 0 };
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        UpdateLaserDistance();

        auto result = CheckParallelStateDecorator(m_stMeasuredata.m_LaserDistance);
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

            log->info("{} tar_pos:{},{},{},{},{},{}", __LINE__, tar_pos[0], tar_pos[1], tar_pos[2],
                tar_pos[3] * 57.3, tar_pos[4] * 57.3, tar_pos[5] * 57.3);
            m_Robot->setLinkMoveAbs(tar_pos, END_VEL_LIMIT);
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
        { 
            log->error("视觉检测无数据，等待下一个周期!!!");
            break;
        }
        UpdateVisionResult(vis_res);
        detectResult = CheckBoardingStateForPositioningDecorator();

        log->info("vis_res.lineDistance: {},{},{},{},{},{}", vis_res.stData.m_LineDistance[0],vis_res.stData.m_LineDistance[1],vis_res.stData.m_LineDistance[2],
                  vis_res.stData.m_LineDistance[3],vis_res.stData.m_LineDistance[4],vis_res.stData.m_LineDistance[5]);
        log->info("check positioning state result: {}", static_cast<int>(detectResult));
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
    static QVector<double> last_tar_position{ 0, 0, 0, 0, 0, 0 };
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        UpdateLaserDistance();
        if (CheckParallelStateDecorator(m_stMeasuredata.m_LaserDistance) == EDetectionInParallelResult::eNoWallDetected)
        {
            log->warn("{} 反馈异常，末端激光偏差过大，状态跳转: 手动--准备", __LINE__);
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            break;
        }

        static QVector<double> tar_position{ 0,0,0,0,0,0 };
        static double tar_pos[6] = { 0,0,0,0,0,0 };
        if (!m_position_motion_flag) {
            log->info("m_stMeasuredata.m_LineDistance:{},{},{},{},{},{}",
                m_stMeasuredata.m_LineDistance[0], m_stMeasuredata.m_LineDistance[1],
                m_stMeasuredata.m_LineDistance[2], m_stMeasuredata.m_LineDistance[3],
                m_stMeasuredata.m_LineDistance[4], m_stMeasuredata.m_LineDistance[5]);
            QVector<Eigen::Matrix4d> Dev_RT = CMeasure::calPoseDeviation(m_stMeasuredata);
            tar_position = m_Robot->getTargetPose(Dev_RT[3]);  // 计算调整量

            for (int i = 0; i < 6; i++) {
                tar_pos[i] = tar_position[i];
            }
            log->info("m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);\ntar_pos:{},{},{},{},{},{}",
                tar_pos[0], tar_pos[1], tar_pos[2], tar_pos[3] * 57.3, tar_pos[4] * 57.3, tar_pos[5] * 57.3);

            m_Robot->setLinkMoveAbs(tar_pos, END_VEL_POSITION);
            stLinkStatus linkstatus = m_Robot->getLinkSta();
            log->info("m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);\nact_pos:{},{},{},{},{},{}",
                tar_pos[0] - linkstatus.stLinkActKin.LinkPos[0], tar_pos[1] - linkstatus.stLinkActKin.LinkPos[1],
                tar_pos[2] - linkstatus.stLinkActKin.LinkPos[2], tar_pos[3] * 57.3 - linkstatus.stLinkActKin.LinkPos[3] * 57.3,
                tar_pos[4] * 57.3 - linkstatus.stLinkActKin.LinkPos[4] * 57.3, tar_pos[5] * 57.3 - linkstatus.stLinkActKin.LinkPos[5] * 57.3);
            m_position_motion_flag = true;
        }
        if (m_LinkStatus.eLinkActState == eLINK_STANDSTILL &&
            m_Robot->isEndReached(tar_position))
        {
            static int cnt = 0;
            if (cnt > 100) {
                cnt = 0;
                m_position_motion_flag = false;
                log->warn("{} 定位运动完成，状态跳转: 定位--检测", __LINE__);
                updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
            }
            cnt++;
        }
        else
        {
            m_Robot->setLinkMoveAbs(tar_pos, END_VEL_LIMIT);
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

void CTask::readyToMagentOnExecutionCommand()
{
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
        m_Comm->SetMagentAction(0, eMag_Off);
        updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        m_Comm->SetMagentAction(0, eMag_Off);
        terminateCommand();
        break;
    }
    default:
    {
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::readyToWeldExecutionCommand()
{
    static int weld{ 0 };
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
        m_Comm->SetMagentAction(0, eMag_Off);
        terminateCommand();
        break;
    }
    default:
    {
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::doingWeldExecutionCommand()
{
    static int weld{ 0 };
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
        if (doWeldAction(1))
        {
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eStopWeld);  // 碰钉结束，跳转到碰钉停止
        }
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        doWeldAction(-1);
        doMagentOff();
        weld = -1;
        break;
    }
    case EExecutionCommand::eStopWeld:
    {
        doWeldAction(-1);
        updateTopAndSubState(ETopState::eDoWeld, ESubState::eStopWeld);
        break;
    }
    case EExecutionCommand::ePause:
    {
        doWeldAction(0);
        updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);
        break;
    }
    default:
    {
        log->warn("{}: {} 执行指令不合法!指令: {}", __LINE__, getCurrentStateString(), ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
    }
    }
}

void CTask::stopWeldExecutionCommand()
{
    static int weld{ 0 };
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
    case EExecutionCommand::eMagentOff:
    {
        doMagentOff();
        weld = -1;
        break;
    }
    case EExecutionCommand::eTerminate:
    {
        m_Comm->SetMagentAction(0, eMag_Off);
        terminateCommand();
        break;
    }
    default:
    {
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
        if (CheckParallelStateDecorator(m_stMeasuredata.m_LaserDistance) == EDetectionInParallelResult::eNoWallDetected)
        {
            log->warn("{} 反馈异常，末端激光偏差过大，状态跳转: 手动--准备", __LINE__);
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            break;
        }

        //调用视觉函数
        VisionResult vis_res = m_vision->getVisResult();
        if (!vis_res.lineStatus)
        { // 视觉检测无数据，等待下一个周期
            break;
        }
        UpdateVisionResult(vis_res);
        result = CheckBoardingStateForFitBoardDecorator();  // Check Board status

        log->info("vis_res.lineDistance: {},{},{},{},{},{}", vis_res.stData.m_LineDistance[0],vis_res.stData.m_LineDistance[1],vis_res.stData.m_LineDistance[2],
                  vis_res.stData.m_LineDistance[3],vis_res.stData.m_LineDistance[4],vis_res.stData.m_LineDistance[5]);
        log->info("check boarding state result: {}", static_cast<int>(result));
#endif
        switch (result)
        {
        case EDetectionInFitBoardResult::eDeviationIsLessThanThreshold:
        {
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eFitBoardFinished);
            break;
        }
        case EDetectionInFitBoardResult::eEndAdjustmentDataIsValid:
        {
            CalculatedAdjustmentForFitBoard();
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eMotion);
            break;
        }
        case EDetectionInFitBoardResult::eDataIsInvalid:
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

void CTask::motionInFitBoardExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        QVector<double> tar_position{ 0,0,0,0,0,0 };
        for (int i = 0; i < m_fit_board_target_pose.size(); i++)
        {
            tar_position[i] = m_fit_board_target_pose.at(i);
        }
        log->info("m_fit_board_target_pose: {}, {}, {}, {}, {}, {}", m_fit_board_target_pose[0], m_fit_board_target_pose[1],
                  m_fit_board_target_pose[2], m_fit_board_target_pose[3],
                  m_fit_board_target_pose[4], m_fit_board_target_pose[5]);
        stLinkStatus linkstatus = m_Robot->getLinkSta();
        log->info("m_Robot->setLinkMoveAbs(m_fit_board_target_pose,END_VEL_LIMIT);\ndifference:{},{},{},{},{},{}",
                  m_fit_board_target_pose[0] - linkstatus.stLinkActKin.LinkPos[0], m_fit_board_target_pose[1] - linkstatus.stLinkActKin.LinkPos[1],
                  m_fit_board_target_pose[2] - linkstatus.stLinkActKin.LinkPos[2], m_fit_board_target_pose[3] * 57.3 - linkstatus.stLinkActKin.LinkPos[3] * 57.3,
                  m_fit_board_target_pose[4] * 57.3 - linkstatus.stLinkActKin.LinkPos[4] * 57.3, m_fit_board_target_pose[5] * 57.3 - linkstatus.stLinkActKin.LinkPos[5] * 57.3);

        m_Robot->setLinkMoveAbs(m_fit_board_target_pose.data(), END_VEL_POSITION);

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
    static int cnt{0};
    switch (m_eexecutionCommand)
    {
    case EExecutionCommand::eNULL:
    {
        // 推缸退出
        m_Comm->SetCylinder(-1);
        if (cnt > 200)  // 10s
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

void CTask::CalculatedAdjustmentForFitBoard()
{
    //计算调整量
    QVector<Eigen::Matrix4d> Dev_RT = CMeasure::calPoseDeviation(m_stMeasuredata, BOARDING_MOTION_QUE.at(m_motion_index));
    QVector<double> tar_position{ 0,0,0,0,0,0 };
    tar_position = m_Robot->getTargetPose(Dev_RT[5]);  // 计算调整量
    log->info("motion index: {}", m_motion_index);

    for (int i = 0; i < m_fit_board_target_pose.size(); i++) 
    {
        m_fit_board_target_pose[i] = tar_position[i];
    }
}

void CTask::UpdateLaserDistance()
{
    //取雷达数据
    QVector<double> LaserDistance = m_Comm->getLasersDistanceBoardingByBojke();

    m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
    m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
    m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
    m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];
}

void CTask::UpdateVisionResult(VisionResult &vis_res)
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
    stateString = itTopState->second.second;

    stateString += "--";

    auto itSubState = SubStateStringMap.find(m_esubState);
    stateString += itSubState->second.second;

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
        default:
        {
            log->error("{} invalid task index: {}", __LINE__, m_manualOperator.TaskIndex);
        }
    }
}