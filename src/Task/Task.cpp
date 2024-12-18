#include "Task.h"

CTask::CTask(ComInterface* comm, CRobot* robot, VisionInterface* vision, QObject* parent)
    : m_Comm(comm), m_Robot(robot), m_vision(vision)
{
    std::memset(&m_LinkStatus, 0, sizeof(m_LinkStatus));
    log = spdlog::get("logger");
}

void CTask::run()
{
    while (this->c_running)
    {
        updateCmdandStatus();
        TranslateManualTaskIndexNumberToCMD();
        stateTransition();
        Sleep(50);
    }
}

void CTask::updateCmdandStatus()
{
    // 更新机器人状态
    m_LinkStatus = m_Robot->getLinkSta();
    m_JointGroupStatus = m_Robot->getJointGroupSta();

    // 更新指令，遥控器+界面
    m_Comm->getManual(m_manualOperator);

    // 根据界面指令修改遥控器指令
    int tmp_val = static_cast<int>(ActionIndex.loadRelaxed());
    switch (ActionIndex.loadRelaxed())
    {
    case 1:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Parallel);
        break;
    case 2:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Positioning);
        break;
    case 4:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::FitBoard);
        break;
    case 6:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Quit);
        break;
    case 7: // 暂停
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Pause);
        break;
    case 8: // 终止
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Terminate);
        break;
    case 9: // 举升
        m_manualOperator.Ready = 1;
        break;
    case 10: // 放钉
        m_manualOperator.Ready = 2;
        break;
    case 11: // 停止
        m_manualOperator.HaltCommand = true;
        break;
    case 12: // 急停
        m_manualOperator.TaskIndex = 128;
        m_manualOperator.StopCommand = true;
        break;
    default:
        break;
    }
    ActionIndex.storeRelaxed(0);
}

void CTask::Manual()
{
    if (m_manualOperator.StopCommand)
    {
        m_Robot->setLinkStop();
        m_preManualOperator = m_manualOperator;
        return;
    }

    if (m_manualOperator.HaltCommand)
    {
        m_Robot->setLinkHalt();
        m_preManualOperator = m_manualOperator;
        return;
    }

    if (std::fabs(m_manualOperator.VechDirect - m_JointGroupStatus[GP::STEER_LEFT_INDEX].Position) > 3)
    {
        const double velocity{ 8.0 };
        // 当前指令和上一个指令的VechDirect都为零，判断条件始终成立
        m_Robot->setJointMoveAbs(GP::STEER_LEFT_INDEX, m_manualOperator.VechDirect, velocity);  // 速度需改为参数
        m_Robot->setJointMoveAbs(GP::STEER_RIGHT_INDEX, m_manualOperator.VechDirect, velocity); // 速度需改为参数
    }

    if (m_manualOperator.bVechFlag || m_manualOperator.bRotateFlag)
    {
        // 计算轮速
        if (fabs(m_manualOperator.VechDirect) > M_PI / 6) // 舵轮角度大于45°时禁止差速转向
        {
            m_manualOperator.VechDirect = 0;
        }
        double vel_left, vel_right;
        vel_left = m_manualOperator.VechVel * 100 - m_manualOperator.RotateVel * 30; // 正转为逆时针
        vel_right = m_manualOperator.VechVel * 100 + m_manualOperator.RotateVel * 30;

        // 解决过速度保护问题
        if (vel_left > 0)
            vel_left = (vel_left > 90.0) ? 90 : vel_left;
        else
            vel_left = (vel_left < -90.0) ? -90 : vel_left;

        if (vel_right > 0)
            vel_right = (vel_right > 90.0) ? 90 : vel_right;
        else
            vel_right = (vel_right < -90.0) ? -90 : vel_right;

        m_Robot->setJointMoveVel(GP::WHEEL_LEFT_INDEX, -vel_left);
        m_Robot->setJointMoveVel(GP::WHEEL_RIGHT_INDEX, -vel_right);
    }
    else
    {
        m_Robot->setJointMoveVel(GP::WHEEL_LEFT_INDEX, 0);
        m_Robot->setJointMoveVel(GP::WHEEL_RIGHT_INDEX, 0);
    }

    std::vector<double> TEMP_LINK_0_JOINT_MAX_VEL(MAX_FREEDOM_LINK, 0.0);
    for (int i = 0; i < TEMP_LINK_0_JOINT_MAX_VEL.size(); i++)
    {
        TEMP_LINK_0_JOINT_MAX_VEL.at(i) = LINK_0_JOINT_MAX_VEL[i] * 0.1;
    }

    if (m_manualOperator.bLinkMoveFlag && m_manualOperator.Ready != 0)
    {
        m_Robot->setLinkHalt();
    }
    else if (m_manualOperator.Ready == 1)
    {
#ifdef TEST_TASK_STATEMACHINE_
#else
        std::vector<double> TEMP_LINK_0_JOINT_MAX_VEL_FOR_READY_POINT(MAX_FREEDOM_LINK, 0.0);
        TEMP_LINK_0_JOINT_MAX_VEL_FOR_READY_POINT = { 3, 3, 3, 1, 0.3, 10, 5, 0.5, 4, 1, 3 };
        m_Robot->setJointGroupMoveAbs(GP::Position_Map[std::make_pair(GP::Working_Scenario, GP::PositionType::Prepare)].value.data(), TEMP_LINK_0_JOINT_MAX_VEL_FOR_READY_POINT.data());
#endif
    }
    else if (m_manualOperator.Ready == 2)
    {
#ifdef TEST_TASK_STATEMACHINE_
#else
        std::vector<double> TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT(MAX_FREEDOM_LINK, 0.0);
        TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT = { 3, 3, 3, 1, 0.3, 10, 5, 0.5, 2, 2, 3 };
        m_Robot->setJointGroupMoveAbs(GP::Position_Map[std::make_pair(GP::Working_Scenario, GP::PositionType::Lift)].value.data(), TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT.data());
#endif
    }
    else if (m_manualOperator.bLinkMoveFlag)
    {
        if (m_manualOperator.bEndMove != m_preManualOperator.bEndMove)
        {
            m_Robot->setLinkHalt();
        }
        else
        {
            std::vector<double> jointvel(20, 0);
            double endvel[6] = { 0, 0, 0, 0, 0, 0 };
            if (m_manualOperator.bEndMove)
            {
                // 末端运动
                for (int i = 0; i < 6; i++)
                {
                    endvel[i] = m_manualOperator.LinkMove[i] * GP::End_Vel_Limit.at(i);
                }
                log->info("{},{}: m_Robot->setLinkMoveVel(endvel): {},{},{},{},{},{}", __FILE__, __LINE__,
                          endvel[0], endvel[1], endvel[2], endvel[3], endvel[4], endvel[5]);
                m_Robot->setLinkMoveVel(endvel);
            }
            else
            {
                // 单轴运动，轴索引待定
                jointvel[0] = m_manualOperator.LinkMove[0] * LINK_0_JOINT_MAX_VEL[0]; // 底升
                jointvel[3] = m_manualOperator.LinkMove[1] * LINK_0_JOINT_MAX_VEL[3]; // 腰
                jointvel[5] = m_manualOperator.LinkMove[2] * LINK_0_JOINT_MAX_VEL[5]; // 大臂俯仰
                jointvel[6] = m_manualOperator.LinkMove[3] * LINK_0_JOINT_MAX_VEL[6]; // 伸缩
                jointvel[8] = m_manualOperator.LinkMove[4] * LINK_0_JOINT_MAX_VEL[8]; // 腕俯仰
                jointvel[9] = m_manualOperator.LinkMove[5] * LINK_0_JOINT_MAX_VEL[9]; // 工装(工具)
                log->info("m_Robot->setJointGroupMoveVel: {},{},{},{},{},{}",
                          jointvel[0], jointvel[3], jointvel[5], jointvel[6], jointvel[8], jointvel[9]);
                m_Robot->setJointGroupMoveVel(jointvel.data());
            }
        }
    }
    else if ((m_manualOperator.Ready == 0 && m_preManualOperator.Ready != 0) || ((m_manualOperator.bLinkMoveFlag == false) && m_preManualOperator.bLinkMoveFlag))
    {
        m_Robot->setLinkHalt();
    }

    switch (m_manualOperator.TaskIndex)
    {
    case stManualOperator::None:
    {
        updateExecutionCommand(EExecutionCommand::eNULL);
        break;
    }
    case stManualOperator::SecondPush:
    {
        double second_push_point = 208.93; // 底部升降位置
        m_Robot->setJointMoveAbs(0, second_push_point, LINK_0_JOINT_MAX_VEL[0]);
        break;
    }
    case stManualOperator::SecondQuit:
    {
        m_Robot->setJointMoveAbs(0, 100.0, LINK_0_JOINT_MAX_VEL[0]);  // 底部升降
        m_Robot->setJointMoveAbs(9, 1000.0, 2);  // 工具升降
        break;
    }
    case stManualOperator::Parallel:
    {
        updateExecutionCommand(EExecutionCommand::eParallel);
        break;
    }
    case stManualOperator::Positioning:
    {
        updateExecutionCommand(EExecutionCommand::ePositioning);
        break;
    }
    case stManualOperator::FitBoard:
    {
        updateExecutionCommand(EExecutionCommand::eFitBoard);
        break;
    }
    case stManualOperator::Quit:
    {
        updateExecutionCommand(EExecutionCommand::eQuit);
        break;
    }
    case stManualOperator::Pause:
    {
        updateExecutionCommand(EExecutionCommand::ePause);
        break;
    }
    case stManualOperator::Terminate:
    {
        updateExecutionCommand(EExecutionCommand::eTerminate);
        break;
    }
    default:
    {
        log->warn("invalid task index: {}", m_manualOperator.TaskIndex);
    }
    }

    m_preManualOperator = m_manualOperator;
}

void CTask::closeThread()
{
    this->c_running = false;
    QThread::wait();
}

// 修改827
int CTask::CheckParallelState(std::vector<double> laserDistance, int max_deviation, int min_deviation, int lift_distance)
{
    // 检查输入数据
    if (laserDistance.size() < 4)
    {
        log->error("激光数据数量输入有误");
        return -1;
    }
    for (int i = 0; i < 4; ++i)
    {
        if (laserDistance[i] > 450 || laserDistance[i] < -3)
        {
            log->error("{}: 激光数据有误,或壁面距离太远, 激光编号: {}, 数值: {}", __LINE__, i, laserDistance.at(i));
            return -1;
        }
    }

    // 计算激光距离最大偏差
    auto max_res = std::max_element(laserDistance.begin(), laserDistance.end());
    auto min_res = std::min_element(laserDistance.begin(), laserDistance.end());
    auto laser_average = std::accumulate(laserDistance.begin(), laserDistance.end(), 0) / laserDistance.size();

    if (*max_res - *min_res > max_deviation && *min_res / *max_res < 0.5)
    {
        log->error("激光距离最大偏差大于{}mm", max_deviation);
        return -1;
    }
    // 判断是否完成调平
    if (std::fabs(*max_res - laser_average) < min_deviation &&
        std::fabs(*min_res - laser_average) < min_deviation &&
        *min_res < lift_distance) // 最大偏差小于阈值
        return 1;
    else
        return 0;
}

EDetectionInParallelResult CTask::CheckParallelStateDecorator()
{
    EDetectionInParallelResult result{ EDetectionInParallelResult::eNoWallDetected };
    std::vector<double> laserDistance(std::begin(m_stMeasuredata.m_LaserDistance), std::end(m_stMeasuredata.m_LaserDistance));

    int res = CheckParallelState(laserDistance, GP::Max_Deviation_In_Parallel, GP::Min_Deviation_In_Parallel, GP::Lift_Distance_In_Parallel);
    switch (res)
    {
    case -1:
    {
        result = EDetectionInParallelResult::eNoWallDetected;
        break;
    }
    case 0:
    {
        result = EDetectionInParallelResult::eDistanceMeetsRequirement;
        break;
    }
    case 1:
    {
        result = EDetectionInParallelResult::eDeviationIsLessThanThreshold;
        break;
    }
    }

    return result;
}

/**
----------*0*--------------------*2*------------------
------------------------------------------------------
---*4*---------------------------------*5*------------
------------------------------------------------------
----------*1*---------------------*3*-----------------
------------------------------------------------------
 */
int CTask::CheckSidelineState()
{
    int re_line = -1;

    // 判断长边合法性
    if ((m_stMeasuredata.m_bLineDistance[0] == false && m_stMeasuredata.m_bLineDistance[2] == false) &&
        (m_stMeasuredata.m_bLineDistance[1] == false && m_stMeasuredata.m_bLineDistance[3] == false))
    {
        log->error("检测到的边线数据长边不满足调整需求");
        return -1;
    }
    // 判断短边合法性
    if ((m_stMeasuredata.m_bLineDistance[4] == false && m_stMeasuredata.m_bLineDistance[5] == false))
    {
        log->error("检测到的边线数据短边不满足调整需求");
        return -1;
    }

    // 计算边线偏差
    double line_dis_1 = ((m_stMeasuredata.m_LineDistance[0] - 15) * m_stMeasuredata.m_bLineDistance[0] - (m_stMeasuredata.m_LineDistance[1] - 15) * m_stMeasuredata.m_bLineDistance[1]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[0]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[1]));
    double line_dis_2 = ((m_stMeasuredata.m_LineDistance[2] - 15) * m_stMeasuredata.m_bLineDistance[2] - (m_stMeasuredata.m_LineDistance[3] - 15) * m_stMeasuredata.m_bLineDistance[3]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[2]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[3]));
    double line_dis_3 = ((m_stMeasuredata.m_LineDistance[4] - 15) * m_stMeasuredata.m_bLineDistance[4] - (m_stMeasuredata.m_LineDistance[5] - 15) * m_stMeasuredata.m_bLineDistance[5]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[4]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[5]));

    if (fabs(line_dis_1) < GP::Line_Deviation_Threshold && fabs(line_dis_2) < GP::Line_Deviation_Threshold && fabs(line_dis_3) < GP::Line_Deviation_Threshold)
    {
        log->info("完成装板定位，边线距离为：{},{},{},{},{},{}", m_stMeasuredata.m_LineDistance[0], m_stMeasuredata.m_LineDistance[1], m_stMeasuredata.m_LineDistance[2], m_stMeasuredata.m_LineDistance[3], m_stMeasuredata.m_LineDistance[4], m_stMeasuredata.m_LineDistance[5]);
        re_line = 1;
    }
    else
    {
        log->error("边线距离需要继续调整，边线距离为：{},{},{},{},{},{}", m_stMeasuredata.m_LineDistance[0], m_stMeasuredata.m_LineDistance[1], m_stMeasuredata.m_LineDistance[2], m_stMeasuredata.m_LineDistance[3], m_stMeasuredata.m_LineDistance[4], m_stMeasuredata.m_LineDistance[5]);
        re_line = 0;
    }

    return re_line;
}

EDetectionInPositioningResult CTask::CheckSidelineStateDecorator()
{
    EDetectionInPositioningResult result;
    int res = CheckSidelineState();

    switch (res)
    {
    case -1:
    {
        result = EDetectionInPositioningResult::eDataIsInvalid;
        break;
    }
    case 0:
    {
        result = EDetectionInPositioningResult::eEndAdjustmentDataIsValid;
        break;
    }
    case 1:
    {
        result = EDetectionInPositioningResult::eDeviationIsLessThanThreshold;
        break;
    }
    default:
    {
        result = EDetectionInPositioningResult::eDataIsInvalid;
        log->error("{} invalid value: {} ", __LINE__, res);
    }
    }

    return result;
}

EDetectionInFitBoardResult CTask::CheckFitBoardState()
{
    EDetectionInFitBoardResult result{ EDetectionInFitBoardResult::eDataIsInvalid };
    std::vector<double> laserDistance(std::begin(m_stMeasuredata.m_LaserDistance), std::end(m_stMeasuredata.m_LaserDistance));

    int res = CheckParallelState(laserDistance, GP::Max_Deviation_In_FitBoard, GP::Min_Deviation_In_FitBoard, GP::Lift_Distance_In_FitBoard);
    switch (res)
    {
    case -1:
    {
        result = EDetectionInFitBoardResult::eDataIsInvalid;
        break;
    }
    case 0:
    {
        result = EDetectionInFitBoardResult::eEndAdjustmentDataIsValid;
        break;
    }
    case 1:
    {
        result = EDetectionInFitBoardResult::eDeviationIsLessThanThreshold;
        break;
    }
    default:
    {
        result = EDetectionInFitBoardResult::eDataIsInvalid;
        log->error("{} invalid value: {} ", __LINE__, res);
    }
    }

    return result;
}

void CTask::TaskTerminate()
{
    // 停止机器人运动
    m_Robot->setLinkHalt();

    // 磁铁脱开
    m_Comm->SetMagentAction(0, eMag_Off);
}