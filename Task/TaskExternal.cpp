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
    //1. 判断机器人是否处于就绪状态 
    bool robotReady = (m_LinkStatus.eLinkActState == eLINK_STANDSTILL);
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
        else if (m_eexecutionCommand == EExecutionCommand::eParallel)
        {
            if(robotReady)
            {
                updateTopAndSubState(ETopState::eFitBoard, ESubState::eDetection);
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
        case EExecutionCommand::eManual:
        {
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
            m_stMeasuredata.m_LaserDistance[0] = laserDistance[0];
            m_stMeasuredata.m_LaserDistance[1] = laserDistance[1];
            m_stMeasuredata.m_LaserDistance[2] = laserDistance[2];
            m_stMeasuredata.m_LaserDistance[3] = laserDistance[3];

            auto detectionResult = CheckParallelStateDecorator(laserDistance);
            log->info("laser distance: {},{},{},{}", laserDistance[0],laserDistance[1],laserDistance[2],laserDistance[3]);
            switch (detectionResult)
            {
                case EDetectionInParallelResult::eDeviationIsLessThanThreshold:
                {
                    log->info("{}: 调平检测结果达到阈值，状态跳转至: 定位--待定位",__LINE__);
                    updateTopAndSubState(ETopState::ePositioning, ESubState::eReadyToPositioning);
                    break;
                }
                case EDetectionInParallelResult::eDistanceMeetsRequirement:
                {
                    log->info("{}: 调平检测结果满足调整条件，状态跳转至: 调平--运动",__LINE__);
                    updateTopAndSubState(ETopState::eParallel, ESubState::eMotion);
                    break;
                }
                case EDetectionInParallelResult::eNoWallDetected:
                {
                    log->info("{}: 调平检测结果不满足调整条件，状态跳转至: 手动--准备",__LINE__);
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
            log->info("{}: 暂停指令，状态跳转至: 调平--待调平",__LINE__);
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
            m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
            m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
            m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
            m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];

            auto result = CheckParallelStateDecorator(LaserDistance);
            if (result == EDetectionInParallelResult::eNoWallDetected)
            {
                log->warn("{} 反馈异常，末端激光偏差过大，状态跳转: 手动",__LINE__);
                updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            }
            else if (result == EDetectionInParallelResult::eDeviationIsLessThanThreshold)
            { 
                log->info("{} 检测结果小于阈值，状态跳转: 调平--检测",__LINE__);
                m_Robot->setLinkHalt();
                updateTopAndSubState(ETopState::eParallel, ESubState::eDetection);
            }
            else
            {
                log->warn("激光测量数据满足调整条件，控制机器人调平运动");
            

                // 计算偏差，控制机器人运动
                QVector<Eigen::Matrix4d> Dev_RT =  CMeasure::calPoseDeviation(m_stMeasuredata);
                QVector<double> tar_position = m_Robot->getTargetPose(Dev_RT[0]);

                double tar_pos[6] ;
                for(int i=0;i<6;i++)
                {
                    tar_pos[i] = tar_position[i];
                }
                log->info("m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);\ntar_pos:{},{},{},{},{},{}", 
                          tar_pos[0],tar_pos[1],tar_pos[2],tar_pos[3] * 57.3,tar_pos[4] * 57.3,tar_pos[5] * 57.3);
                m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            log->info("{} 终止指令，状态跳转: 手动--准备", __LINE__);
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
            log->info("{} 定位指令，状态跳转: 定位--检测", __LINE__);
            updateTopAndSubState(ETopState::ePositioning, ESubState::eDetection);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            log->info("{} 终止指令，状态跳转: 手动--准备", __LINE__);
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

            //调用视觉函数
            VisionResult vis_res = m_vision->getVisResult();
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
                    log->warn("{} 定位检测结果小于阈值，状态跳转: 贴合--待贴合",__LINE__);
                    updateTopAndSubState(ETopState::eFitBoard, ESubState::eReadyToFitBoard);
                    break;
                }
                case EDetectionInPositioningResult::eEndAdjustmentDataIsValid:
                {
                    log->warn("{} 定位检测结果为有效数据，状态跳转: 定位--运动",__LINE__);
                    updateTopAndSubState(ETopState::ePositioning, ESubState::eMotion);
                    break;
                }
                case EDetectionInPositioningResult::eDataIsInvalid:
                {
                    log->warn("{} 定位检测结果为无效数据，状态跳转: 手动--准备",__LINE__);
                    updateTopAndSubState(ETopState::eManual, ESubState::eReady);
                    break;
                }
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            log->info("{} 终止指令，状态跳转: 手动--准备", __LINE__);
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
            m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
            m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
            m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
            m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];


            if (CheckParallelStateDecorator(LaserDistance) == EDetectionInParallelResult::eNoWallDetected)
            {
                log->warn("{} 反馈异常，末端激光偏差过大，状态跳转: 手动--准备", __LINE__);
                updateTopAndSubState(ETopState::eManual, ESubState::eReady);
                break;
            }
            static QVector<double> tar_position{0,0,0,0,0,0};
            static double tar_pos[6] = {0,0,0,0,0,0};
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
        log->info("{} 终止指令，状态跳转: 手动--准备", __LINE__);
            m_position_motion_flag = false;
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            m_position_motion_flag = false;
            log->info("{} 停止运动，状态跳转: 定位--待定位", __LINE__);
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
//    static EExecutionCommand static_command {EExecutionCommand::eNULL};
//    if (static_command != EExecutionCommand::eNULL)
//    {
//        m_eexecutionCommand = static_command;
//    }
    static int magent{0};
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            if (magent == 1)
            {
                if (doMagentOn())
                {
                    magent = 0;
                    log->warn("{} 吸合指令运行完成，状态跳转: 碰钉--待碰钉", __LINE__);
                    updateTopAndSubState(ETopState::eDoWeld, ESubState::eReadyToDoWeld);
                }
            }


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
//            log->info("磁铁吸合指令，状态跳转: 碰钉--待碰钉");

            doMagentOn();
            magent = 1;
            break;
        }
        case EExecutionCommand::eQuit:
        {
            //doMagentOff();
            m_Comm->SetMagentAction(0,eMag_Off);
            log->warn("{} 退出指令，状态跳转: 退出--退出中", __LINE__);
            updateTopAndSubState(ETopState::eQuit, ESubState::eQuiting);
            break;
        }
        case EExecutionCommand::eTerminate:
        {
        log->info("{} 终止指令，状态跳转: 手动--准备", __LINE__);
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
            // log->info("碰钉-待碰钉状态下 空指令，不执行任何操作");
             if (weld == -1)
            {
                if (doMagentOff())
                {
                    weld = 0;
                    log->warn("{} 脱开指令运行完成，状态跳转: 待吸合--空", __LINE__);
                    updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
                }
            }
            break;
        }
        case EExecutionCommand::eAutoWeld:
        {
            log->info("{} 自动碰钉指令，状态跳转: 碰钉-碰钉中", __LINE__);
            updateTopAndSubState(ETopState::eDoWeld, ESubState::eDoingWeld);
            break;
        }
        case EExecutionCommand::eMagentOff:
        {
//            log->info("磁铁脱开，状态跳转: 待吸合");

            doMagentOff();
            weld = -1;
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            m_Comm->SetMagentAction(0,eMag_Off);
            log->info("{} 终止指令执行，磁铁脱开，状态跳转: 手动--准备", __LINE__);
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
            // log->info("碰钉--碰钉中 空指令，执行自动碰钉，完成碰钉后状态跳转: 退出--退出中");
            if (weld == -1)
            {
                if (doMagentOff())
                {
                    weld = 0;
            log->info("{} 终止指令执行，磁铁脱开，状态跳转: 手动--准备", __LINE__);
                    terminateCommand();
                }
                break;
            }
            if (doWeldAction(1))
            {
            log->info("{} 碰钉结束，状态跳转: 碰钉--停止碰钉", __LINE__);
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
                    log->info("{} 脱开指令运行结束，状态跳转: 待吸合--空", __LINE__);
                    updateTopAndSubState(ETopState::eReadToMagentOn, ESubState::eNULL);
                }
            }
            // log->info("碰钉--停止状态下 空指令，不做任何操作");
            break;
        }
        case EExecutionCommand::eMagentOff:
        {
            // log->info("磁铁脱开，状态跳转: 待吸合");
            doMagentOff();
            weld = -1;
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            m_Comm->SetMagentAction(0,eMag_Off);
                    log->info("{} 终止指令执行，磁铁脱开，状态跳转: 手动--准备", __LINE__);
            terminateCommand();
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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
            // LIINKHALT
            break;
        }
        case EExecutionCommand::eTerminate:
        {
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            break;
        }
        default:
        {
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::detectionInFitBoardExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            EDetectionInPositioningResult result;  // CheckBoarding
            switch (result)
            {
                case EDetectionInPositioningResult::eDeviationIsLessThanThreshold:
                {
                    updateTopAndSubState(ETopState::eFitBoard, ESubState::eFitBoardFinished);
                    break;
                }
                case EDetectionInPositioningResult::eEndAdjustmentDataIsValid:
                {
                    updateTopAndSubState(ETopState::eFitBoard, ESubState::eMotion);
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
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
        }
    }
}

void CTask::motionInFitBoardExecutionCommand()
{
    switch (m_eexecutionCommand)
    {
        case EExecutionCommand::eNULL:
        {
            // 运动-->完成运动
            // m_Robot->moveAbs(pos, vel); if (isreachedendpos && standstill) 状态跳转: 贴合--检测
            updateTopAndSubState(ETopState::eFitBoard, ESubState::eDetection);
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
            log->warn("{} {}: 执行指令不合法!指令: {}",__FILE__, __LINE__, ExecutionCommandStringMap.find(m_eexecutionCommand)->second);
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

            break;
        }
        case EExecutionCommand::eTerminate:
        {
            updateTopAndSubState(ETopState::eManual, ESubState::eReady);
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
            m_Robot->setJointMoveAbs(9,1100,7);
            //m_Robot->setJointGroupMoveAbs(Postion_Quit,JOINT_VEL_LIMIT);
            //if(m_Robot->isJointReached(Postion_Quit_qv))
            if(m_JointGroupStatus[9].Position < 1110)
            {
            log->info("{} 退出指令运行结束，状态跳转: 手动--准备", __LINE__);
                updateTopAndSubState(ETopState::eManual, ESubState::eReady);
            }
            break;
        }
        case EExecutionCommand::eTerminate:
        {
        log->info("{} 终止指令执行，磁铁脱开，状态跳转: 手动--准备", __LINE__);
            terminateCommand();
            break;
        }
        case EExecutionCommand::ePause:
        {
            log->info("{} 停止运动，状态跳转: 退出--暂停", __LINE__);
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
        log->info("{} 终止指令执行，状态跳转: 手动--准备", __LINE__);
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

    log->info("{} 状态转换: {}--{} ==> {}--{}", __LINE__, TopStateStringMap[m_etopState], SubStateStringMap[m_esubState], TopStateStringMap[topState], SubStateStringMap[subState]);

    m_etopState = topState;
    m_esubState = subState;
}

void CTask::updateExecutionCommand(EExecutionCommand executionCommand)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    // log->info("{} 指令转换: {} ==> {}", __LINE__, ExecutionCommandStringMap[m_eexecutionCommand], ExecutionCommandStringMap[executionCommand]);

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