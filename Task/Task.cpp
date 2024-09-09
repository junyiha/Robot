#include "Task.h"

CTask::CTask(ComInterface* comm,CRobot* robot,VisionInterface* vision,QObject *parent)
{
    m_Comm = comm;
    m_Robot = robot;
    m_vision = vision;

    std::memset(&m_LinkStatus, 0, sizeof(m_LinkStatus));
    std::memset(&m_Manual, 0, sizeof(stManualData));
    std::memset(&m_preManual, 0, sizeof(stManualData));
    //std::memset(&m_JointGroupStatus,0,sizeof(st_AxisGroupRead));

    for(int i = 0;i<7;i++)
    {
        m_TargetDeviation.push_back(Eigen::Matrix4d::Identity());
    }


    m_AutoWorking = false;
    m_Step = eEnd;

    log = spdlog::get("logger");
    m_bMagnetOn = false;

    log->info("碰钉机器人--任务模块构造完成...");
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

        // SemiAutoProgrcess();

        // TranslateNumberToCMD();
        TranslateManualTaskIndexNumberToCMD();
        stateTransition();

//        if(m_Manual.Auto)
//        {
//            SemiAutoProgrcess();
//        }else
//        {
//            Manual();
//            m_preManual = m_Manual;
//        }
//        m_preManualOperator = m_manualOperator;
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
    //m_manualOperator.Ready = 0;
    //m_manualOperator.TaskIndex = 0;
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
            m_manualOperator.TaskIndex = static_cast<int>(stManualOperator::ETaskIndex::DoWeld);
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



void CTask::SemiAutoProgrcess()
{
    QVector<double> LaserDistance(4);
    std::fill(LaserDistance.begin(), LaserDistance.end(), 0.0); // 初始化LaserDistance为0

    QVector<Eigen::Matrix4d> Dev_RT(6);
    for (auto& mat : Dev_RT) {
        mat.setIdentity(); // 如果想要设置为单位矩阵
        // 或者，如果你想要一个所有元素都为0的矩阵，你可以使用：
        // mat.setZero();
    }

    QVector<double> tar_position(6);
    std::fill(tar_position.begin(), tar_position.end(), 0.0); // 初始化tar_position为0

    static double tar_pos[6] = {0};
    double tool_tar = 0;

    // double Postion_Home[10]    =    {10,0,0,-90.04,-0.52,0,1210, 0, 0, 0}; //碰钉准备位置
    // double Postion_Prepare[10]    = {800,0,0,-90.04,0.12,0.27,1300, 0, 0, 0}; //碰钉准备位置
    // QVector<double> Postion_Home_qv = {10,0,0,-90.04,-0.52,0,1210, 0, 0, 0}; //碰钉准备位置
    // QVector<double> Postion_Prepare_qv = {800,0,0,-90.04,0.12,0.27,1300,0, 0, 0}; //碰钉准备位置

//    double Postion_Home[7]    =    {650,-7,-54,-1.11,0.09,-0.05,1210}; //碰钉准备位置
//    double Postion_Prepare[7]    = {816,-7,-54,-1.11,0.09,-0.05,1300}; //碰钉准备位置
//    QVector<double> Postion_Home_qv = {650,-7,-54,-1.11,0.09,-0.05,1210}; //碰钉准备位置
//    QVector<double> Postion_Prepare_qv = {816,-7,-54,-1.11,0.09,-0.05,1300}; //碰钉准备位置

    VisionResult vis_res; // 视觉检测结果

    static quint16 cnt_15 = 0;
    static quint16 cnt_25 = 0;

    if(ActionIndex.loadRelaxed() != 15)
    {
        cnt_15 = 0;
    }
    //ActionIndex状态改变-日志输出
    static int preActionIndex = 0;
    if(preActionIndex != ActionIndex.loadRelaxed()){
        log->info(("action_index changed from  " +  GetEnumName_action_index(preActionIndex) + "  ------->  " + GetEnumName_action_index(ActionIndex.loadRelaxed())).c_str());
    }
    preActionIndex = ActionIndex.loadRelaxed();

    //action_index 按照枚举变量重新定义
    switch(ActionIndex.loadRelaxed())
    {
    //=====================0~10 :空闲、急停、按暂停 等 ==========================
    case 0:
        break;
    case 1: //急停
        m_Robot->setLinkStop();
        ActionIndex.storeRelaxed(0);

    case 3: //停止
        m_Robot->setLinkHalt();
        ActionIndex.storeRelaxed(0);
        break;

    //=====================10~19 :举升调平 ==========================
    case 10: //举升准备位置
        if(this->m_bMagnetOn){break;}
        m_Robot->setJointGroupMoveAbs(Postion_Home,JOINT_VEL_LIMIT);
        if(m_Robot->isJointReached(Postion_Home_qv))
        {
            ActionIndex.storeRelaxed(0);
        }
        break;

    case 12: //举升到调平位置
        //检测高差
        if(this->m_bMagnetOn){break;}
        m_Robot->setJointGroupMoveAbs(Postion_Prepare,JOINT_VEL_LIMIT);
        if(m_Robot->isJointReached(Postion_Prepare_qv))
        {
            ActionIndex.storeRelaxed(0);
        }

        break;

    case 15: //调平
        if(this->m_bMagnetOn){break;}
        LaserDistance = m_Comm->getLasersDistance();

        m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
        m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
        m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
        m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];

        Dev_RT =  CMeasure::calPoseDeviation(m_stMeasuredata);
        if(Dev_RT[6](1,0) == false)break;
        tar_position = m_Robot->getTargetPose(Dev_RT[0]);

        for(int i=0;i<6;i++)
        {
            tar_pos[i] = tar_position[i];
        }

        if(Dev_RT[6](0,2) < 50) //点激光一致性小于50mm
        {
            if(Dev_RT[6](0,0)<Distance_Lift+10)
            {
                m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);

                if(Dev_RT[6](0,2)<3) //四个点激光差值1mm
                {
                    ActionIndex.storeRelaxed(0);
                    log->info("完成调平，偏差：{}",Dev_RT[6](0,2));
                }

            }else{
               m_Robot->setLinkHalt();
               ActionIndex.storeRelaxed(0);//
               log->warn("末端距离过大，请检查设备和环境");
            }
        }
        else//偏差过大
        {
            m_Robot->setLinkHalt();
            ActionIndex.storeRelaxed(0);
            log->warn("末端激光偏差过大，请检查设备和环境");
        }
        cnt_15 ++;
        if(cnt_15 > 100)
        {
            log->warn("调平中止");
            ActionIndex.storeRelaxed(0);
        }
        break;

    case 16: //举升到对边位置
        if(this->m_bMagnetOn){break;}
        LaserDistance = m_Comm->getLasersDistance();

        m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
        m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
        m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
        m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];
        Dev_RT =  CMeasure::calPoseDeviation(m_stMeasuredata);
        if(Dev_RT[6](0,0) < 10 || Dev_RT[6](0,0)>50)
        {
            log->warn("距离有误，当前距离{}",Dev_RT[6](0,0));
            ActionIndex.storeRelaxed(0);
        }
        tool_tar = Dev_RT[6](0,0) -Distance_work + m_JointGroupStatus[6].Position;

        m_Robot->setJointMoveAbs(6,tool_tar,5); //末端举升， 5mm/s
        ActionIndex.storeRelaxed(0);
        break;

    //=====================20~29:边线对齐=====================
    case 20: //获取边线数据，并计算调整量
        if(this->m_bMagnetOn){break;}
        vis_res = m_vision->getVisResult();
        if(vis_res.status) {  //调用视觉函数
            std::copy(std::begin(vis_res.stData.m_LineDistance) , std::end(vis_res.stData.m_LineDistance), m_stMeasuredata.m_LineDistance);
            std::copy(std::begin(vis_res.stData.m_bLineDistance), std::end(vis_res.stData.m_bLineDistance), m_stMeasuredata.m_bLineDistance);

            LaserDistance = m_Comm->getLasersDistance();
            m_stMeasuredata.m_LaserDistance[0] = LaserDistance[0];
            m_stMeasuredata.m_LaserDistance[1] = LaserDistance[1];
            m_stMeasuredata.m_LaserDistance[2] = LaserDistance[2];
            m_stMeasuredata.m_LaserDistance[3] = LaserDistance[3];


            log->info("有效性 \nline1:{}\nline2:{}\nline3:{}\nline4{}",m_stMeasuredata.m_bLineDistance[0],m_stMeasuredata.m_bLineDistance[1],m_stMeasuredata.m_bLineDistance[2],m_stMeasuredata.m_bLineDistance[3]);
            log->info("边线值 \nline1:{}\nline2:{}\nline3:{}\nline4{}",m_stMeasuredata.m_LineDistance[0],m_stMeasuredata.m_LineDistance[1],m_stMeasuredata.m_LineDistance[2],m_stMeasuredata.m_LineDistance[3]);

            Dev_RT =  CMeasure::calPoseDeviation(m_stMeasuredata);
            PrintTargetPos(3,Dev_RT);

            //数据有效性检测
            if(Dev_RT[6](1,3) == false){
                log->warn("边线数据有误，请检查设备--The boundary line data is incorrect. Please check the device.");
                ActionIndex.storeRelaxed(0);
                break;
            }

            //计算调整量
            tar_position = m_Robot->getTargetPose(Dev_RT[3]);
            for(int i=0;i<6;i++)
            {
                tar_pos[i] = tar_position[i];
            }
            QVector<st_ReadAxis> stJointstatus = m_Comm->getLinkJointStatus(0); //link轴组数据

            log->info("\n end_pos 1:{}\n end_pos 2:{}\n end_pos 3:{}\n end_pos 4:{}\n end_pos 5:{}\n end",stJointstatus[0].Position,stJointstatus[1].Position,stJointstatus[2].Position,stJointstatus[3].Position,stJointstatus[4].Position);
            log->info("\n tar_pos 1:{}\n tar_pos 2:{}\n tar_pos 3:{}\n tar_pos 4:{}\n tar_pos 5:{}\n tar_pos 6:{}\n ",tar_pos[0],tar_pos[1],tar_pos[2],tar_pos[3],tar_pos[4],tar_pos[5]);
            log->info("对边线调整量计算完毕--The calculation of the adjustment amount for the opposite sideline has been completed.");

            ActionIndex.storeRelaxed(25);
        }
        break;

    case 25: //调整偏差
        if(this->m_bMagnetOn){break;}
        m_Robot->setLinkMoveAbs(tar_pos,END_VEL_LIMIT);

        cnt_25 ++;

        log->info("机器人正在进行边线调整...");
        if(m_LinkStatus.eLinkActState == eLINK_STANDSTILL && m_Robot->isEndReached(tar_position) == true )
        {
            log->info("完成边线调整");
            cnt_25 = 0;
            for(int i=0;i<6;i++)
            {
                tar_pos[i] = 0.0;
            }
            ActionIndex.storeRelaxed(28);
        }

        if(cnt_25 > 200){
            log->warn("边线调整中止");
            cnt_25 = 0;
            for(int i=0;i<6;i++)
            {
                tar_pos[i] = 0.0;
            }
            ActionIndex.storeRelaxed(0);
        }

        break;

    case 28: //检测调整结果
    {
        if(this->m_bMagnetOn){break;}
        const double staLineDistance = 15;
        const double deltaLineDistance = 1.5;
        vis_res = m_vision->getVisResult();
        if(vis_res.status) //调用视觉函数
        {
            //输出偏差 emit
            bool b_line_error = false;
            for(int i=0;i<4;i++){
                if(!vis_res.stData.m_bLineDistance[i])
                {
                    b_line_error = false;
                    log->warn("边线{}数据有误，请再次“对齐边线”",i);
                }

                if(vis_res.stData.m_LineDistance[i] < staLineDistance + deltaLineDistance && vis_res.stData.m_LineDistance[i] > staLineDistance - deltaLineDistance)
                {
                    b_line_error = false;
                    log->info("边线{  }偏差合理，当前距离：{  }",i,vis_res.stData.m_LineDistance[i]);
                }
            }
            ActionIndex.storeRelaxed(0);
        }
        break;
    }
        //=====================30~50:碰钉动作=====================
    case 30: //磁铁吸合
        if(doMagentOn() == true)
        {
            m_bMagnetOn = true;
            ActionIndex.storeRelaxed(0);
            log->info("完成磁铁吸合操作");
            qDebug("确认磁铁吸合状况，及螺柱、磁环安装状态"); //弹窗提升
        }
        break;

    case 32: //磁铁脱开
        if(doMagentOff() == true)
        {
            m_bMagnetOn = false;
            ActionIndex.storeRelaxed(0);
            log->info("完成磁铁脱开操作");
        }
        break;

    case 40: //自动碰钉
        if (doWeldAction(1) == true)
        {
            ActionIndex.storeRelaxed(0);
            log->info("完成自动碰钉作业");
        }
        break;

    case 42: //自动碰钉暂停            
        ActionIndex.storeRelaxed(0);
        break;

    case 44: //结束/中止碰钉
        doWeldAction(-1);
        ActionIndex.storeRelaxed(0);
        break;
    }


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

    if (std::fabs(m_manualOperator.VechDirect -  m_JointGroupStatus[STEER_LEFT_INDEX].Position) > 3)
    {
        // 当前指令和上一个指令的VechDirect都为零，判断条件始终成立
         m_Robot->setJointMoveAbs(STEER_LEFT_INDEX, m_manualOperator.VechDirect,10);//速度需改为参数
        log->info("m_Robot->setJointMoveAbs(STEER_LEFT_INDEX, {}", m_manualOperator.VechDirect);

         m_Robot->setJointMoveAbs(STEER_RIGHT_INDEX,m_manualOperator.VechDirect,10);//速度需改为参数
        log->info("m_Robot->setJointMoveAbs(STEER_RIGHT_INDEX, {}", m_manualOperator.VechDirect);
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

        log->info("{},{}: m_Robot->setJointMoveVel(WHEEL_LEFT_INDEX, {});", __FILE__,__LINE__,vel_left);
         m_Robot->setJointMoveVel(WHEEL_LEFT_INDEX, -vel_left);
        log->info("{},{}: m_Robot->setJointMoveVel(WHEEL_RIGHT_INDEX, {});", __FILE__,__LINE__,vel_right);
         m_Robot->setJointMoveVel(WHEEL_RIGHT_INDEX, -vel_right);
    }else
    {
        m_Robot->setJointMoveVel(WHEEL_LEFT_INDEX, 0);
        m_Robot->setJointMoveVel(WHEEL_RIGHT_INDEX, 0);
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
        log->info("{},{}: m_Robot->setLinkMoveAbs(Postion_Home,END_VEL_LIMIT);", __FILE__,__LINE__);

        //m_Comm->setLinkJointMoveAbs(0, Postion_Prepare,TEMP_LINK_0_JOINT_MAX_VEL.data());
        log->info("Postion_Prepare: {},{},{},{},{},{},{},{},{},{}\nTEMP_LINK_0_JOINT_MAX_VEL: {},{},{},{},{},{},{},{},{},{}",
                  Postion_Prepare[0],Postion_Prepare[1],Postion_Prepare[2],Postion_Prepare[3],Postion_Prepare[4],Postion_Prepare[5],Postion_Prepare[6],Postion_Prepare[7],Postion_Prepare[8],Postion_Prepare[0],
                  TEMP_LINK_0_JOINT_MAX_VEL[1],TEMP_LINK_0_JOINT_MAX_VEL[2],TEMP_LINK_0_JOINT_MAX_VEL[3],TEMP_LINK_0_JOINT_MAX_VEL[4],TEMP_LINK_0_JOINT_MAX_VEL[5],TEMP_LINK_0_JOINT_MAX_VEL[6],TEMP_LINK_0_JOINT_MAX_VEL[7],TEMP_LINK_0_JOINT_MAX_VEL[8],TEMP_LINK_0_JOINT_MAX_VEL[9]);
        m_Robot->setJointGroupMoveAbs(Postion_Prepare,TEMP_LINK_0_JOINT_MAX_VEL.data());
    }
    else if (m_manualOperator.Ready == 2)
    {
        // 移动到放钉位置
        log->info("{},{}: m_Robot->setLinkMoveAbs(Postion_Prepare,END_VEL_LIMIT);", __FILE__,__LINE__);

        //m_Comm->setLinkJointMoveAbs(0, Postion_Home,TEMP_LINK_0_JOINT_MAX_VEL.data());
        m_Robot->setJointGroupMoveAbs(Postion_Home,TEMP_LINK_0_JOINT_MAX_VEL.data());
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
                    endvel[i] = m_manualOperator.LinkMove[i]*END_VEL_LIMIT[i];
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
    else if((!m_manualOperator.bLinkMoveFlag  && !m_manualOperator.Ready) && (m_preManualOperator.bLinkMoveFlag || m_preManualOperator.Ready))
    {
        m_Robot->setLinkHalt();
    }

    if(m_manualOperator.TaskIndex != 0)
    {
        log->info("TaskIndex:{}",m_manualOperator.TaskIndex);
    }
    switch (m_manualOperator.TaskIndex)
    {
        case stManualOperator::None:
        {
            updateExecutionCommand(EExecutionCommand::eNULL);
            break;
        }
        case stManualOperator::Parallel:
        {
            log->info("updateExecutionCommand(EExecutionCommand::eParallel);");
            updateExecutionCommand(EExecutionCommand::eParallel);
            break;
        }
        case stManualOperator::Positioning:
        {
            log->info("updateExecutionCommand(EExecutionCommand::ePositioning);");
            updateExecutionCommand(EExecutionCommand::ePositioning);
            break;
        }
        case stManualOperator::DoWeld:
        {
            log->info("updateExecutionCommand(EExecutionCommand::eAutoWeld);");
            updateExecutionCommand(EExecutionCommand::eAutoWeld);
            break;
        }
        case stManualOperator::MagentOn:
        {
            log->info("updateExecutionCommand(EExecutionCommand::eMagentOn);");
            updateExecutionCommand(EExecutionCommand::eMagentOn);
            break;
        }
        case stManualOperator::MagentOff:
        {
            log->info("updateExecutionCommand(EExecutionCommand::eMagentOff);");
            updateExecutionCommand(EExecutionCommand::eMagentOff);
            break;
        }
        case stManualOperator::Quit:
        {
            log->info("updateExecutionCommand(EExecutionCommand::eQuit);");
            updateExecutionCommand(EExecutionCommand::eQuit);
            break;
        }
        case stManualOperator::Pause:
        {
            log->info("updateExecutionCommand(EExecutionCommand::ePause);");
            updateExecutionCommand(EExecutionCommand::ePause);
            break;
        }
        case stManualOperator::Terminate:
        {
            log->info("updateExecutionCommand(EExecutionCommand::eTerminate);");
            updateExecutionCommand(EExecutionCommand::eTerminate);
            break;
        }
        default:
        {
            // 非法任务指令
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

void CTask::PrintTargetPos(uint index,QVector<Eigen::Matrix4d> m_TargetDeviation)
{
    qDebug()<<"----------------------PrintTargetPos : [ "+QString::number(index)+" ]---------------------------";
    for(int i=0;i<4;++i){
        qDebug()<<m_TargetDeviation[index](i,0)<<" || "<<m_TargetDeviation[index](i,1)<<" || "<<m_TargetDeviation[index](i,2)<<" || "<<m_TargetDeviation[index](i,3)<<" || ";
    }
    qDebug()<<"\n\n";
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
        if(laserDistance[i] > 300 || laserDistance[i] < 0) {
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
        log->info("激光距离{},最大偏差小于{}，完成调平", Distance_Lift, PARRALLE_DISTANCE);
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
    double line_dis_1 = (m_stMeasuredata.m_LineDistance[0] * m_stMeasuredata.m_bLineDistance[0] -
                         m_stMeasuredata.m_LineDistance[1] * m_stMeasuredata.m_bLineDistance[1]) /
                         (static_cast<int>(m_stMeasuredata.m_bLineDistance[0]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[1]));

    double line_dis_2 = (m_stMeasuredata.m_LineDistance[2] * m_stMeasuredata.m_bLineDistance[2] -
                         m_stMeasuredata.m_LineDistance[3]*m_stMeasuredata.m_bLineDistance[3]) /
                         (static_cast<int>(m_stMeasuredata.m_bLineDistance[2]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[3]));

    double line_dis_3 = (m_stMeasuredata.m_LineDistance[4] * m_stMeasuredata.m_bLineDistance[4] -
                         m_stMeasuredata.m_LineDistance[5] * m_stMeasuredata.m_bLineDistance[5]) /
                         (static_cast<int>(m_stMeasuredata.m_bLineDistance[4]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[5]));

    if(fabs(line_dis_1) < LINE_DEVIATION_THRESHOLD
    && fabs(line_dis_2) < LINE_DEVIATION_THRESHOLD
    && fabs(line_dis_3) < LINE_DEVIATION_THRESHOLD)
    {
        log->info("完成调平，边线距离为：{},{},{},{},{},{}",m_stMeasuredata.m_LineDistance[0],m_stMeasuredata.m_LineDistance[1],m_stMeasuredata.m_LineDistance[2],m_stMeasuredata.m_LineDistance[3],m_stMeasuredata.m_LineDistance[4],m_stMeasuredata.m_LineDistance[5]);
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