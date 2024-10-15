#include "Task.h"

CTask::CTask(ComInterface* comm,CRobot* robot,VisionInterface* vision,QObject *parent)
{
    m_Comm = comm;
    m_Robot = robot;
    m_vision = vision;

    std::memset(&m_LinkStatus, 0, sizeof(m_LinkStatus));
    std::memset(&m_Manual, 0, sizeof(stManualData));
    std::memset(&m_preManual, 0, sizeof(stManualData));

    for(int i = 0;i<7;i++)
    {
        m_TargetDeviation.push_back(Eigen::Matrix4d::Identity());
    }

    log = spdlog::get("logger");
    m_bMagnetOn = false;
}

void CTask::run()
{
    //初始参数初始化
    log->info("Task 启动运行....");

    //周期函数
    while(this->c_running)
    {
        log->trace("Task 启动运行...");

        updateCmdandStatus();

        TranslateManualTaskIndexNumberToCMD();

        stateTransition();

        Sleep(50);
    }

    log->info("Task 退出运行....");
}

void CTask::updateCmdandStatus()
{
    // 更新机器人状态
    m_LinkStatus = m_Robot->getLinkSta();
    m_JointGroupStatus = m_Robot->getJointGroupSta();

    // 更新指令，遥控器+界面
    m_Comm->getManual(m_manualOperator);

    //根据界面指令修改遥控器指令
    int tmp_val = static_cast<int>(ActionIndex.loadRelaxed());
    switch(ActionIndex.loadRelaxed())
    {
    case 1:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Parallel);
        break;
    case 2:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Positioning);
        break;
    case 3:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::MagentOn);
        break;
    case 4:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::FitBoard);
        break;
    case 5:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::MagentOff);
        break;
    case 6:
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Quit);
        break;
    case 7: //暂停
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Pause);
        break;
    case 8: //终止
        m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::Terminate);
        break;

    case 9: //举升
        m_manualOperator.Ready = 1;
        break;
    case 10://放钉
        m_manualOperator.Ready = 2;
        break;
    case 11://停止
        m_manualOperator.HaltCommand = true;
        break;
    case 12://急停
        m_manualOperator.TaskIndex = 128;
        m_manualOperator.StopCommand = true;
        break;
    default:
        break;
    }
    ActionIndex.storeRelaxed(0);

    this->mutex_read.lock();
    _stMeasuredata = m_stMeasuredata;
    this->mutex_read.unlock();
}

void CTask::Manual()
{
    if (m_manualOperator.StopCommand)
    {
        log->info("{},{}: m_Robot->setLinkStop();", __FILE__,__LINE__);
        m_Robot->setLinkStop();
        m_preManualOperator = m_manualOperator ;
        return;
    }

    if (m_manualOperator.HaltCommand)
    {
        log->info("{},{}: m_Robot->setLinkHalt();", __FILE__,__LINE__);
        m_Robot->setLinkHalt();
        m_preManualOperator = m_manualOperator ;
        return;
    }

    if (std::fabs(m_manualOperator.VechDirect -  m_JointGroupStatus[GP::STEER_LEFT_INDEX].Position) > 3)
    {
        // 当前指令和上一个指令的VechDirect都为零，判断条件始终成立
         m_Robot->setJointMoveAbs(GP::STEER_LEFT_INDEX, m_manualOperator.VechDirect,10);//速度需改为参数
        log->info("m_Robot->setJointMoveAbs(GP::STEER_LEFT_INDEX, {}", m_manualOperator.VechDirect);

         m_Robot->setJointMoveAbs(GP::STEER_RIGHT_INDEX,m_manualOperator.VechDirect,10);//速度需改为参数
        log->info("m_Robot->setJointMoveAbs(GP::STEER_RIGHT_INDEX, {}", m_manualOperator.VechDirect);
    }

    double vel_left,vel_right ;

    if (m_manualOperator.bVechFlag || m_manualOperator.bRotateFlag)
    {
        //计算轮速
        if(fabs(m_manualOperator.VechDirect)>M_PI/6)//舵轮角度大于45°时禁止差速转向
        {
            m_manualOperator.VechDirect = 0;
        }
        // 速度待修改 2024.09.03
        vel_left = m_manualOperator.VechVel * 100 - m_manualOperator.RotateVel * 30; //正转为逆时针
        vel_right = m_manualOperator.VechVel * 100 + m_manualOperator.RotateVel * 30;

        log->info("{},{}: m_Robot->setJointMoveVel(GP::WHEEL_LEFT_INDEX, {});", __FILE__,__LINE__,vel_left);
         m_Robot->setJointMoveVel(GP::WHEEL_LEFT_INDEX, -vel_left);
        log->info("{},{}: m_Robot->setJointMoveVel(GP::WHEEL_RIGHT_INDEX, {});", __FILE__,__LINE__,vel_right);
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
        log->info("{},{}: m_Robot->setLinkHalt();", __FILE__,__LINE__);
        m_Robot->setLinkHalt();
    }
    else if (m_manualOperator.Ready == 1)
    {
        // 移动到举升位置
        log->info("{},{}: m_Robot->setLinkMoveAbs(Postion_Prepare,GP::End_Vel_Limit.data());", __FILE__,__LINE__);
#ifdef TEST_TASK_STATEMACHINE_
#else
        std::vector<double> TEMP_LINK_0_JOINT_MAX_VEL_FOR_READY_POINT(MAX_FREEDOM_LINK, 0.0);
        TEMP_LINK_0_JOINT_MAX_VEL_FOR_READY_POINT = {3, 3, 3, 1, 0.3, 10, 5, 0.5, 4, 1, 3};
        m_Robot->setJointGroupMoveAbs(GP::Prepare_Position.data(), TEMP_LINK_0_JOINT_MAX_VEL_FOR_READY_POINT.data());
#endif
    }
    else if (m_manualOperator.Ready == 2)
    {
        // 移动到装扮位置
        log->info("{},{}: m_Robot->setLinkMoveAbs(Postion_Home,GP::End_Vel_Limit.data());", __FILE__,__LINE__);
#ifdef TEST_TASK_STATEMACHINE_
#else
        std::vector<double> TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT(MAX_FREEDOM_LINK, 0.0);
        TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT = {3, 3, 3, 1, 0.3, 10, 5, 0.5, 2, 6, 3};
        m_Robot->setJointGroupMoveAbs(GP::Home_Position.data(), TEMP_LINK_0_JOINT_MAX_VEL_FOR_SET_POINT.data());
#endif
    }
    else if (m_manualOperator.bLinkMoveFlag)
    {
        if (m_manualOperator.bEndMove != m_preManualOperator.bEndMove)
        {
            log->info("{},{}: m_Robot->setLinkHalt();", __FILE__,__LINE__);
             m_Robot->setLinkHalt();
        }
        else
        {
            std::vector<double> jointvel(20, 0);
            double endvel[6] ={0,0,0,0,0,0};
            if (m_manualOperator.bEndMove)
            {
                // 末端运动
                for(int i= 0;i<6;i++)
                {
                    endvel[i] = m_manualOperator.LinkMove[i]* GP::End_Vel_Limit.at(i);
                }
                log->info("{},{}: m_Robot->setLinkMoveVel(endvel): {},{},{},{},{},{}", __FILE__,__LINE__,
                endvel[0],endvel[1],endvel[2],endvel[3],endvel[4],endvel[5]);
                 m_Robot->setLinkMoveVel(endvel);
            }
            else
            {
                // 单轴运动，轴索引待定
                jointvel[0] = m_manualOperator.LinkMove[0] * LINK_0_JOINT_MAX_VEL[0];		// 底升
                jointvel[3] = m_manualOperator.LinkMove[1] * LINK_0_JOINT_MAX_VEL[3];	    // 腰
                jointvel[5] = m_manualOperator.LinkMove[2] * LINK_0_JOINT_MAX_VEL[5];		// 大臂俯仰
                jointvel[6] = m_manualOperator.LinkMove[3] * LINK_0_JOINT_MAX_VEL[6];		// 伸缩
                jointvel[8] = m_manualOperator.LinkMove[4] * LINK_0_JOINT_MAX_VEL[8];		// 腕俯仰
                jointvel[9] = m_manualOperator.LinkMove[5] * LINK_0_JOINT_MAX_VEL[9];		// 工装(工具)
                log->info("m_Robot->setJointGroupMoveVel: {},{},{},{},{},{}",
                                jointvel[0],jointvel[3],jointvel[5],jointvel[6],jointvel[8],jointvel[9]);
                 m_Robot->setJointGroupMoveVel(jointvel.data());
            }
        }
    }
    else if( (m_manualOperator.Ready == 0 &&  m_preManualOperator.Ready !=0) || ((m_manualOperator.bLinkMoveFlag == false) && m_preManualOperator.bLinkMoveFlag))
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
            double second_push_point = 208.93;  // 底部升降位置
            m_Robot->setJointMoveAbs(0, second_push_point, LINK_0_JOINT_MAX_VEL[0]);
            break;
        }
        case stManualOperator::SecondQuit:
        {
            double second_push_point = 100.0;  // 底部升降位置
            m_Robot->setJointMoveAbs(0, second_push_point, LINK_0_JOINT_MAX_VEL[0]);
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
        case stManualOperator::DoWeld:
        {
            updateExecutionCommand(EExecutionCommand::eAutoWeld);
            break;
        }
        case stManualOperator::MagentOn:
        {
            updateExecutionCommand(EExecutionCommand::eMagentOn);
            break;
        }
        case stManualOperator::MagentOff:
        {
            updateExecutionCommand(EExecutionCommand::eMagentOff);
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

bool CTask::doWeldAction(qint8 execute)
{
    static quint8  index_tool = 1; //执行焊枪编号范围1~5
    static quint8  index_act  = 0;
    static quint8  time_cnt = 0; //周期计数，控制动作间隔

    //=====================结束碰钉 ==========================
    if(execute == -1)
    {
        unsigned char tem = ActionList[index_act]&0b00100011;//关闭打磨、碰钉、定位气缸、打磨降
        tem |= 0b00100000; //碰钉枪下降
        m_Comm->SetToolsAction(index_tool,(E_WeldAction)tem);
        m_Comm->SetToolsAction(11-index_tool,(E_WeldAction)tem);
        m_Comm->SetGunConnect(0);//关闭接触器

        index_tool = 1;
        index_act  = 0;
        time_cnt = 0;
        log->info("结束碰钉作业");
        return true;
    }

    //=====================暂停碰钉 ==========================
    static quint32 pause_cnt = 0;
    if(execute == 0)
     {
         pause_cnt++;
         if(pause_cnt>600)//暂停时间过长，停止
         {
             unsigned char tem = ActionList[index_act]&0b10111111;//关闭打磨
             m_Comm->SetToolsAction(index_tool,(E_WeldAction)tem);
             m_Comm->SetToolsAction(11-index_tool,(E_WeldAction)tem);
             m_Comm->SetGunConnect(0);//关闭接触器

             pause_cnt = 0;
             log->warn("暂停时间过长，关闭打磨及接触器");
         }
         log->info("自动碰钉暂停");
         return true;
     }

    //=============== 执行10把焊枪轮次焊接1~5,6~10 ==============
    if(index_act<ActionList.size())
    {
        if(time_cnt == 0)
        {
            //执行动作
            m_Comm->SetToolsAction(index_tool,ActionList[index_act]); //1~5号枪动作
            m_Comm->SetToolsAction(11-index_tool,ActionList[index_act]);//6~10号枪动作
            log->info("焊枪{}执行动作:{} {}",index_tool,index_act,ActionName[index_act]);

            //在特定动作连接焊枪
            if(ActionList[index_act] == eWeld_Up )
            {
                m_Comm->SetGunConnect(index_tool);
                m_Comm->SetGunConnect(11-index_tool);
                log->debug("接触器吸合" );
            }
            if(ActionList[index_act] == eWeld_Down)
            {
                m_Comm->SetGunConnect(0);
                log->debug("接触器断开" );
            }
            time_cnt++;


        }else
        {
            //计数等待
            time_cnt ++;

            if(time_cnt > ActionTime[index_act])//等待结束，进入下一个动作
            {
                index_act++;
                time_cnt = 0;
            }
        }

        return  false;

    }else
    {
        //完成所有动作，切换到下一把焊枪
        log->info("完成焊枪：{}",index_tool);

        m_Comm->SetToolsAction(index_tool,eInitAction); //1~5号枪动作
        m_Comm->SetToolsAction(11-index_tool,eInitAction);//6~10号枪动作

        index_act = 0;
        index_tool++;

        if(index_tool > 5) //所有焊枪动作完成
        {
            index_tool = 1;
            log->info("所有焊枪动作完成");
            return true;
        }

        return false;
    }

}

bool CTask::doMagentOff()
{
    static quint8 act_index = 0;
    static quint8 time_cnt  = 0; //周期计数，控制动作间隔
    if(act_index == 0){
        log->info("doMagentOff--time_cnt:{}",time_cnt);
    }

    switch (act_index) {
    case 0://磁铁断电
        if(time_cnt == 0)
        {
            m_Comm->SetMagentAction(0,eMag_Off);
            time_cnt ++;
            log->info("eMag_Off");
        }
        else
        {
            //计数等待
            time_cnt ++;
            if(time_cnt > 50)//等待结束，进入下一个动作
            {
                act_index = 1;
                time_cnt = 0;
            }
        }
        return false;

    case 1:  //推缸缩
        if(time_cnt == 0)
        {
            m_Comm->SetMagentAction(0,eMag_Off);
            m_Comm->SetMagentAction(0,eMag_Down);
            time_cnt ++;
            log->info("eMag_Down");
        }
        else
        {
            //计数等待
            time_cnt ++;
            if(time_cnt > 200)//等待结束，进入下一个动作
            {
                act_index = 0;
                time_cnt = 0;
                return true;
            }
        }
        return false;

    default:
        log->warn("无效指令");
        act_index = 0;
        time_cnt = 0;
        return false;
    }
}

bool CTask::doMagentOn()
{
    static quint8 act_index = 0;
    static quint8 time_cnt  = 0; //周期计数，控制动作间隔

    if(act_index == 0){
        log->info("doMagentOn--time_cnt:{}",time_cnt);
    }

    switch (act_index) {
    case 0:  //磁铁举升
        if(time_cnt == 0)
        {
            m_Comm->SetMagentAction(0,eMag_Off);
            time_cnt ++;
            log->info("eMag_Off");
        }
        else
        {
            //计数等待
            time_cnt ++;
            if(time_cnt > 20)//等待结束，进入下一个动作
            {
                act_index = 1;
                time_cnt = 0;
            }
        }
        return false;
    case 1:
        if(time_cnt == 0)
        {
            m_Comm->SetMagentAction(0,eMag_Up);
            log->info("eMag_Up");
            time_cnt ++;
        }
        else
        {
            //计数等待
            time_cnt ++;
            if(time_cnt > 100)//等待结束，进入下一个动作
            {
                act_index = 2;
                time_cnt = 0;
            }
        }
        return false;
    case 2://磁铁吸合
        m_Comm->SetMagentAction(0,eMag_On);
        act_index = 0;
        time_cnt = 0;
        log->info("eMag_On");
        return true;
    default:
        act_index = 0;
        time_cnt = 0;
        return false;
    }

}

stMeasureData CTask::getStMeasureData()
{
    stMeasureData re;
    this->mutex_read.lock();
    re = m_stMeasuredata;
    this->mutex_read.unlock();

    return re;
}

void CTask::closeThread() {
    this->c_running = false;
    QThread::wait();

}

//修改827
int CTask::CheckParallelState(QVector<double> laserDistance)
{
    //检查输入数据
    if(laserDistance.size() < 4) {
        log->error("激光数据数量输入有误");
        return -1;
    }
    for(int i=0;i<4;++i) {
        if(laserDistance[i] > 300 || laserDistance[i] < -3) {
            log->error("{}激光数据有误,或壁面距离太远",i);
            return -1;
        }
    }

    //计算激光距离最大偏差
    double maxDistance = 0;
    double minDistance = 1000;
    for(int i=0;i<4;++i) {
        maxDistance = maxDistance>laserDistance[i]?maxDistance:laserDistance[i];
        minDistance = minDistance<laserDistance[i]?minDistance:laserDistance[i];
    }
    if(maxDistance - minDistance>50 && minDistance/maxDistance < 0.5)
    {
        log->error("激光距离最大偏差大于50mm");
        return -1;
    }
    //判断是否完成调平
    if(maxDistance - minDistance< PARRALLE_DISTANCE && minDistance < Distance_Lift)  //最大偏差小于阈值
    {
        //log->info("激光距离{},最大偏差小于{}，完成调平", Distance_Lift, PARRALLE_DISTANCE);
        return 1;
    }else
    {
        return 0;
    }
}

EDetectionInParallelResult CTask::CheckParallelStateDecorator(QVector<double> laserDistance)
{
    EDetectionInParallelResult result;
    int res = CheckParallelState(laserDistance);
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

EDetectionInParallelResult CTask::CheckParallelStateDecorator(double laserDistance[])
{
    //int size = sizeof(laserDistance) / sizeof(laserDistance[0]);
    QVector<double> LaserDistance(laserDistance, laserDistance + 4);

    return CheckParallelStateDecorator(LaserDistance);
}

/**
----------*0*--------------------*2*------------------
------------------------------------------------------
---*4*---------------------------------*5*------------
------------------------------------------------------
----------*1*---------------------*3*-----------------
------------------------------------------------------
 */
int CTask::CheckPositionState()
{
    //视觉检测结果有效性检测
    if((m_stMeasuredata.m_bLineDistance[0]||m_stMeasuredata.m_bLineDistance[1])  == false
    || (m_stMeasuredata.m_bLineDistance[2]||m_stMeasuredata.m_bLineDistance[3] ) == false
    || (m_stMeasuredata.m_bLineDistance[4]||m_stMeasuredata.m_bLineDistance[5] )== false)
    {
        log->error("line918检测到的边线数据不满足调整需求");
        return -1;
    }

    double Ref_Distance = 15;

    //边线偏差是否小于阈值
    double line_dis_1 = ((m_stMeasuredata.m_LineDistance[0]-15) * m_stMeasuredata.m_bLineDistance[0] -
                         (m_stMeasuredata.m_LineDistance[1]-15) * m_stMeasuredata.m_bLineDistance[1]) /
                         (static_cast<int>(m_stMeasuredata.m_bLineDistance[0]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[1]));

    double line_dis_2 = ((m_stMeasuredata.m_LineDistance[2]-15) * m_stMeasuredata.m_bLineDistance[2] -
                         (m_stMeasuredata.m_LineDistance[3]-15)*m_stMeasuredata.m_bLineDistance[3]) /
                         (static_cast<int>(m_stMeasuredata.m_bLineDistance[2]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[3]));

//    double line_dis_3 = (m_stMeasuredata.m_LineDistance[4] * m_stMeasuredata.m_bLineDistance[4] -
//                         m_stMeasuredata.m_LineDistance[5] * m_stMeasuredata.m_bLineDistance[5]) /
//                         (static_cast<int>(m_stMeasuredata.m_bLineDistance[4]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[5]));
    double line_dis_3 = 0 ;
    if(m_stMeasuredata.m_bLineDistance[4])
    {
         line_dis_3 = m_stMeasuredata.m_LineDistance[4];
    }



    log->info("line_dis:{},{},{}",line_dis_1,line_dis_2,line_dis_3);
    if(fabs(line_dis_1) < LINE_DEVIATION_THRESHOLD
    && fabs(line_dis_2) < LINE_DEVIATION_THRESHOLD
    && fabs(line_dis_3-41) < LINE_DEVIATION_THRESHOLD)
    {
        log->info("完成对边，边线距离为：{},{},{},{},{},{}",m_stMeasuredata.m_LineDistance[0],m_stMeasuredata.m_LineDistance[1],m_stMeasuredata.m_LineDistance[2],m_stMeasuredata.m_LineDistance[3],m_stMeasuredata.m_LineDistance[4],m_stMeasuredata.m_LineDistance[5]);
        return 1;
    }
    else
    {
        log->error("边线距离不满足碰钉要求，边线距离为：{},{},{},{},{},{}",m_stMeasuredata.m_LineDistance[0],m_stMeasuredata.m_LineDistance[1],m_stMeasuredata.m_LineDistance[2],m_stMeasuredata.m_LineDistance[3],m_stMeasuredata.m_LineDistance[4],m_stMeasuredata.m_LineDistance[5]);
        return 0;
    }

}

EDetectionInPositioningResult CTask::CheckPositionStateDecorator()
{
    EDetectionInPositioningResult result;
    int res = CheckPositionState();

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
    }

    return result;
}

void CTask::TaskTerminate()
{
    // 停止机器人运动
    m_Robot->setLinkHalt();

    // 磁铁脱开
    m_Comm->SetMagentAction(0,eMag_Off);
}

int CTask::CheckBoardingState(uint& motion_index)
{

    int re_line = -1;
    int re_laser = -1;

    // 判断长边合法性
    if((m_stMeasuredata.m_bLineDistance[0]==false && m_stMeasuredata.m_bLineDistance[2]==false) |
       (m_stMeasuredata.m_bLineDistance[1]==false && m_stMeasuredata.m_bLineDistance[3]==false)
    ){
        log->error("检测到的边线数据长边不满足调整需求");
        return -1;
    }
    // 判断短边合法性
    if((m_stMeasuredata.m_bLineDistance[4]==false && m_stMeasuredata.m_bLineDistance[5]==false)){
        log->error("检测到的边线数据短边不满足调整需求");
        return -1;
    }

//    bool tmp1 = m_stMeasuredata.m_bLineDistance[0] || (m_stMeasuredata.m_bLineDistance[1] == false);
//    bool tmp2 = m_stMeasuredata.m_bLineDistance[2] || (m_stMeasuredata.m_bLineDistance[3] == false);
//    bool tmp3 = m_stMeasuredata.m_bLineDistance[5] || (m_stMeasuredata.m_bLineDistance[4] == false);
//    if (tmp1 || tmp2 || tmp3)
//    {
//        log->error("检测到的边线数据不满足调整需求");
//        return -1;
//    }
    //视觉检测结果有效性检测
//    if ((m_stMeasuredata.m_bLineDistance[0] || (m_stMeasuredata.m_bLineDistance[1] == false))  //
//        || (m_stMeasuredata.m_bLineDistance[2] || (m_stMeasuredata.m_bLineDistance[3] == false))
//        || (m_stMeasuredata.m_bLineDistance[5] || (m_stMeasuredata.m_bLineDistance[4] == false)))  //
//    {
//        log->error("检测到的边线数据不满足调整需求");
//        return -1;
//    }

    //计算边线偏差
    double line_dis_1 = ((m_stMeasuredata.m_bLineDistance[0] - 15) * m_stMeasuredata.m_bLineDistance[0] - (m_stMeasuredata.m_bLineDistance[1] - 15) * m_stMeasuredata.m_bLineDistance[1]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[0] )+ static_cast<int>(m_stMeasuredata.m_bLineDistance[1]));
    double line_dis_2 = ((m_stMeasuredata.m_bLineDistance[2] - 15) * m_stMeasuredata.m_bLineDistance[2] - (m_stMeasuredata.m_bLineDistance[3] - 15) * m_stMeasuredata.m_bLineDistance[3]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[2] )+ static_cast<int>(m_stMeasuredata.m_bLineDistance[3]));
    double line_dis_3 = ((m_stMeasuredata.m_bLineDistance[4] - 15) * m_stMeasuredata.m_bLineDistance[4] - (m_stMeasuredata.m_bLineDistance[5] - 15) * m_stMeasuredata.m_bLineDistance[5]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[4] )+ static_cast<int>(m_stMeasuredata.m_bLineDistance[5]));

    if (fabs(line_dis_1) < LINE_DEVIATION_THRESHOLD
        && fabs(line_dis_2) < LINE_DEVIATION_THRESHOLD
        && fabs(line_dis_3) < LINE_DEVIATION_THRESHOLD)
    {
        log->info("完成贴合定位，边线距离为：{},{},{},{},{},{}", m_stMeasuredata.m_LineDistance[0], m_stMeasuredata.m_LineDistance[1], m_stMeasuredata.m_LineDistance[2], m_stMeasuredata.m_LineDistance[3], m_stMeasuredata.m_LineDistance[4], m_stMeasuredata.m_LineDistance[5]);
        re_line = 1;
    }
    else if (fabs(line_dis_1) < LINE_DEVIATION_BOARDING_THRESHOLD
        && fabs(line_dis_2) < LINE_DEVIATION_BOARDING_THRESHOLD
        && fabs(line_dis_3) < LINE_DEVIATION_BOARDING_THRESHOLD)
    {
        log->error("边线距离需要继续调整，边线距离为：{},{},{},{},{},{}", m_stMeasuredata.m_LineDistance[0], m_stMeasuredata.m_LineDistance[1], m_stMeasuredata.m_LineDistance[2], m_stMeasuredata.m_LineDistance[3], m_stMeasuredata.m_LineDistance[4], m_stMeasuredata.m_LineDistance[5]);
        re_line = 0;
    }

//    //计算激光距离最大偏差
//    double laser[4] = { 0,0,0,0 };
//    for (int i = 0; i < 4; ++i)
//    {
//        if (m_stMeasuredata.m_bLaserProfile[0] == true)
//        {
//            laser[i] = m_stMeasuredata.m_LaserGapHeight[i];
//        }
//        else if (m_stMeasuredata.m_bLaserDistance[1] == true)
//        {
//            laser[i] = m_stMeasuredata.m_LaserDistance[i];
//        }
//        else
//        {
//            log->error("激光{}距离缺失,停止贴合", i);
//            return -1;
//        }
//
//    }
    double maxDistance = 0;
    double minDistance = 1000;
    for (int i = 0; i < 4; ++i) {
        maxDistance = maxDistance > m_stMeasuredata.m_LaserDistance[i] ? maxDistance : m_stMeasuredata.m_LaserDistance[i];
        minDistance = minDistance < m_stMeasuredata.m_LaserDistance[i] ? minDistance : m_stMeasuredata.m_LaserDistance[i];
    }
    if (maxDistance - minDistance > 20)
    {
        log->error("激光距离最大偏差大于20mm,停止贴合");
        return -1;
    }
    else if (maxDistance - minDistance < 2 && maxDistance < 2)
    {
        log->info("激光距离满足贴合要求");
        re_laser = 1;
    }

    //根据板面距离确定目标贴合距离
    motion_index = 6;
    for (int i = 0; i < BOARDING_MOTION_QUE.size(); ++i)
    {
        if (BOARDING_MOTION_QUE[i] + 2 < minDistance)
        {
            motion_index = i;
            re_line = 0;
            log->info("贴合调整目标距离为：{}", BOARDING_MOTION_QUE[i]);
            break;
        }
    }

    if (re_line == 1 && re_laser == 1)
    {
        log->info("完成贴合作业，可以进行拧螺丝作业");
        return 1;
    }

    return 0;
}

int CTask::CheckBoardingStateForPositioning()
{
    int re_line = -1;

    // 判断长边合法性
    if((m_stMeasuredata.m_bLineDistance[0]==false && m_stMeasuredata.m_bLineDistance[2]==false) &&
       (m_stMeasuredata.m_bLineDistance[1]==false && m_stMeasuredata.m_bLineDistance[3]==false)
            ){
        log->error("检测到的边线数据长边不满足调整需求");
        return -1;
    }
    // 判断短边合法性
    if((m_stMeasuredata.m_bLineDistance[4]==false && m_stMeasuredata.m_bLineDistance[5]==false)){
        log->error("检测到的边线数据短边不满足调整需求");
        return -1;
    }

    //计算边线偏差
    double line_dis_1 = ((m_stMeasuredata.m_LineDistance[0] - 15) * m_stMeasuredata.m_bLineDistance[0] - (m_stMeasuredata.m_LineDistance[1] - 15) * m_stMeasuredata.m_bLineDistance[1]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[0] )+ static_cast<int>(m_stMeasuredata.m_bLineDistance[1]));
    double line_dis_2 = ((m_stMeasuredata.m_LineDistance[2] - 15) * m_stMeasuredata.m_bLineDistance[2] - (m_stMeasuredata.m_LineDistance[3] - 15) * m_stMeasuredata.m_bLineDistance[3]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[2] )+ static_cast<int>(m_stMeasuredata.m_bLineDistance[3]));
    double line_dis_3 = ((m_stMeasuredata.m_LineDistance[4] - 15) * m_stMeasuredata.m_bLineDistance[4] - (m_stMeasuredata.m_LineDistance[5] - 15) * m_stMeasuredata.m_bLineDistance[5]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[4] )+ static_cast<int>(m_stMeasuredata.m_bLineDistance[5]));

    log->info("{} line_dis_1: {}, line_dis_2: {}, line_dis_3: {}", __LINE__, line_dis_1, line_dis_2, line_dis_3);

    if (fabs(line_dis_1) < LINE_DEVIATION_THRESHOLD
        && fabs(line_dis_2) < LINE_DEVIATION_THRESHOLD
        && fabs(line_dis_3) < LINE_DEVIATION_THRESHOLD)
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

EDetectionInPositioningResult CTask::CheckBoardingStateForPositioningDecorator()
{
    EDetectionInPositioningResult result;
    int res = CheckBoardingStateForPositioning();

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
    }

    return result;
}

int CTask::CheckBoardingStateForFitBoard(uint &motion_index)
{
    int exit_flag = 1;
    int re_laser = -1;

    double maxDistance = 0;
    double minDistance = 1000;
    for (int i = 0; i < 4; ++i) {
        maxDistance = maxDistance > m_stMeasuredata.m_LaserDistance[i] ? maxDistance : m_stMeasuredata.m_LaserDistance[i];
        minDistance = minDistance < m_stMeasuredata.m_LaserDistance[i] ? minDistance : m_stMeasuredata.m_LaserDistance[i];
    }
    if (maxDistance - minDistance > 20)
    {
        log->error("激光距离最大偏差大于20mm,停止贴合");
        return -1;
    }
    else if (maxDistance - minDistance < 6 && maxDistance < 6)  // 5
    {
        log->info("激光距离满足贴合要求");
        re_laser = 1;
    }

    //根据板面距离确定目标贴合距离
    motion_index = 6;
    for (int i = 0; i < BOARDING_MOTION_QUE.size(); ++i)
    {
        if (BOARDING_MOTION_QUE[i] + 2 < minDistance)
        {
            motion_index = i;
            exit_flag = 0;
            log->info("贴合调整目标距离为：{}", BOARDING_MOTION_QUE[i]);
            break;
        }
    }

    log->info("motion index: {}, min distance: {}", motion_index, minDistance);

    if (exit_flag == 1 && re_laser == 1)
    {
        log->info("完成贴合作业，可以进行拧螺丝作业");
        return 1;
    }

    return 0;
}

EDetectionInFitBoardResult CTask::CheckBoardingStateForFitBoardDecorator()
{
    EDetectionInFitBoardResult result;
    int res = CheckBoardingStateForFitBoard(m_motion_index);

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
    }

    return result;
}

EDetectionInPositioningResult CTask::CheckBoardingStateDecorator()
{
    EDetectionInPositioningResult result;
    int res = CheckBoardingState(m_motion_index);

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
    }

    return result;
}