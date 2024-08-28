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

        SemiAutoProgrcess();

        stateTransition();

//        if(m_Manual.Auto)
//        {
//            SemiAutoProgrcess();
//        }else
//        {
//            Manual();
//            m_preManual = m_Manual;
//        }
        Sleep(50);
    }

    log->info("Task 退出运行....");
}

void CTask::updateCmdandStatus()
{
    m_LinkStatus = m_Robot->getLinkSta();
    m_JointGroupStatus = m_Robot->getJointGroupSta();
    m_Comm->getManual(m_Manual);

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

    double Postion_Home[7]    =    {10,0,0,-90.04,-0.52,0,1210}; //碰钉准备位置
    double Postion_Prepare[7]    = {800,0,0,-90.04,0.12,0.27,1300}; //碰钉准备位置
    QVector<double> Postion_Home_qv = {10,0,0,-90.04,-0.52,0,1210}; //碰钉准备位置
    QVector<double> Postion_Prepare_qv = {800,0,0,-90.04,0.12,0.27,1300}; //碰钉准备位置

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

//bool CTask::DoAction(int stage)
//{
//    static bool enabledetect = false;
//    static bool isdetected = false;
//    qDebug()<<"=============== enabledetect ==============  "<<enabledetect;
//    qDebug()<<"=============== isdetected ================   "<<isdetected;
//    stMeasureData stMeasuredata;

//    if(ActionIndex.load() == -1 || ActionIndex.load() == -2){//如果中途暂停，暂停时enabledetect以及isdetected的值如果没重置的话，有可能下次执行其他步骤的时候，程序会跳过发送检测请求以及结果校验部分。
//        enabledetect = false;
//        isdetected = false;
//        m_Robot->setLinkHalt();
//        ActionIndex.store(-2);
//        for (auto& matrix : m_TargetDeviation) {
//            matrix.setZero();
//        }
//        return false;
//    }

//    //--------------------------------发送检测请求--------------------------------
//    if(enabledetect == false)
//    {
//        switch(stage){
//        case -2:{
//            ActionIndex.store(-10);
//            m_Robot->setLinkHalt();
//            return false;
//        }
//        case -1:{
//            qDebug()<<"";
//            ActionIndex.store(-10);
//            m_Robot->setLinkStop();
//            return false;
//        }
//        //  【0】 调平，调板间隙
//        case 0:{
//            qDebug()<< "stage 0 is beginning";
//            this->m_Comm->getLasersStatus(&stMeasuredata);//点激光结果获取
//            this->m_vision->getDetectResult(LIDAR, &stMeasuredata);// 雷达结果获取
//            qDebug()<< "stage 0 has been finished";
//        };break;

//        //  【1】 对圆孔
//        case 1:{
//            qDebug()<< "stage 1 is beginning";
//            this->m_Comm->getLasersStatus(&stMeasuredata);//点激光结果获取
//            this->m_vision->getDetectResult(HOLE, &stMeasuredata);// 圆孔结果获取
//            qDebug()<< "stage 1 has been finished";
//        };break;

//        //  【2】 进孔
//        case 2:{
//            qDebug()<< "stage 2 is beginning";
//            this->m_Comm->getLasersStatus(&stMeasuredata);//点激光结果获取
//            QVector<st_ReadAxis> stJointstatus = m_Comm->getLinkJointStatus(0); //link轴组数据
//            QVector<st_ReadAxis> stJointPitch = m_Comm->getLinkJointStatus(1);//腰部俯仰数据
//            stJointstatus.append(stJointPitch.back());
//            stMeasuredata.m_JointStatusTool = stJointstatus[TOOL_INDEX].Position;
//            qDebug()<< "stage 2 has been finished";
//        };break;

//        //  【3】 对边
//        case 3:{
//            qDebug()<< "stage 3 is beginning";
//            this->m_Comm->getLasersStatus(&stMeasuredata);//点激光结果获取
//            this->m_vision->getDetectResult(LINE,&stMeasuredata);// 边线结果获取
//            qDebug()<< "stage 3 has been finished";
//        };break;

//        //  【4】 贴合壁面
//        case 4:{
//            qDebug()<< "stage 4 is beginning";
//            this->m_Comm->getLasersStatus(&stMeasuredata);//点激光结果获取
//            QVector<st_ReadAxis> stJointstatus = m_Comm->getLinkJointStatus(0); //link轴组数据
//            QVector<st_ReadAxis> stJointPitch = m_Comm->getLinkJointStatus(1);//腰部俯仰数据
//            stJointstatus.append(stJointPitch.back());
//            stMeasuredata.m_JointStatusTool = stJointstatus[TOOL_INDEX].Position;
//            qDebug()<< "stage 4 has been finished";
//        };break;

//        //  【5】 对边
//        case 5:{
//            qDebug()<< "stage 5 is beginning";
//            this->m_Comm->getLasersStatus(&stMeasuredata);//点激光结果获取
//            this->m_vision->getDetectResult(LINE,&stMeasuredata);// 边线结果获取
//            qDebug()<< "stage 5 has been finished";
//        };break;

//        default:{
//            qDebug()<<"检测请求错误";
//        };break;

//        }

//        qDebug()<< "【begin】CMeasure::calPoseDeviation(stMeasuredata);//计算机器人调整量";

//        m_TargetDeviation = CMeasure::calPoseDeviation(stMeasuredata);//计算机器人调整量
//        m_Robot->update_PoseDev(m_TargetDeviation[stage]);//更新目标位姿数据
//        qDebug()<< "【end】CMeasure::calPoseDeviation(stMeasuredata);//计算机器人调整量";

//        this->mutex_write.lock();
//        this->m_stMeasuredata = stMeasuredata;
//        this->mutex_write.unlock();

//        enabledetect = true;
//        return false;
//    }

//    //--------------------------------判断依据视觉结果计算出的调整量是否可以信任--------------------------------
//    qDebug()<<"PrintTargetPos(6,m_TargetDeviation)";
//    this->PrintTargetPos(6,m_TargetDeviation);

//    if(isdetected == false){
//        switch(stage){
//        //  【0】 调平，调板间隙
//        case 0:{
//            if(m_TargetDeviation[6](1,0)==1)isdetected = true;
//            else isdetected = false;
//        };break;

//        //  【1】 对圆孔
//        case 1:{
//            if(m_TargetDeviation[6](1,1)==1)isdetected = true;
//            else isdetected = false;
//        };break;

//        //  【2】 进孔 Z方向平动 50
//        case 2:{
//            if(m_TargetDeviation[6](1,2)==1)isdetected = true;
//            else isdetected = false;

//        };break;

//        //  【3】 对边
//        case 3:{
//            if(m_TargetDeviation[6](1,3)==1)isdetected = true;
//            else isdetected = false;
//        };break;

//        //  【4】 贴合壁面  Z方向平动 30
//        case 4:{
//            if(m_TargetDeviation[6](2,0)==1)isdetected = true;
//            else isdetected = false;

//        };break;

//        //  【5】 对边
//        case 5:{
//            if(m_TargetDeviation[6](2,1)==1)isdetected = true;
//            else isdetected = false;
//        };break;

//        default:{
//            qDebug()<<"有视觉结果返回值 但无有效调整量";
//        };break;
//        }
//    }
//    if(isdetected == false){
//         enabledetect = false;
//         return false;
//    }
//    else{
//    }

//    //--------------------------------根据检测结果移动机器人--------------------------------
//    if(stage == 0 || stage == 1 || stage == 3 || stage == 5){
//        qDebug()<<QString("【Type1】DoAction( %1)").arg(stage);
//        m_Robot->setLinkMoveAbs(stage,END_VEL_LIMIT);
//    }
//    else if(stage == 2){
//        qDebug()<<QString("【Type2】DoAction( %1)").arg(stage);
//        m_Robot->setJointMoveAbs(TOOL_INDEX,stMeasuredata.m_JointStatusTool + m_TargetDeviation[stage](2,3),3);
//    }
//    else if(stage == 4){
//        qDebug()<<QString("【Type3】DoAction( %1)").arg(stage);
//        m_Robot->setJointMoveAbs(TOOL_INDEX,stMeasuredata.m_JointStatusTool + m_TargetDeviation[stage](2,3),3);
//    }else{
//        ActionIndex.store(-2);
//        m_Robot->setLinkHalt();
//    }

//    //--------------------------------检测机器人运动是否完成--------------------------------
//    bool finishFlag = false;

//    if(stage == 0 || stage == 1 || stage == 3 || stage == 5){
//        if(m_LinkStatus.eLinkActState == eLINK_STANDSTILL
//                &&m_LinkStatus.stLinkActKin.bDone == true){
//            finishFlag = true;
//        }
//    }else if(stage == 2 || stage == 4){
//        QVector<st_ReadAxis> stJointstatus = m_Comm->getLinkJointStatus(0); //link轴组数据
//        QVector<st_ReadAxis> stJointPitch = m_Comm->getLinkJointStatus(1);//腰部俯仰数据
//        stJointstatus.append(stJointPitch.back());

//        if(stage == 2){
//            if(fabs(stJointstatus[TOOL_INDEX].Position - (stMeasuredata.m_JointStatusTool + m_TargetDeviation[stage](2,3))) < 0.05 ){
//                finishFlag = true;
//            }else{
//                qDebug()<<"*****************  【 stage  "<<stage<<" is not finished 】*****************";
//            }
//        }
//        else if(stage == 4){
//            if(fabs(stJointstatus[TOOL_INDEX].Position - (stMeasuredata.m_JointStatusTool + m_TargetDeviation[stage](2,3))) < 0.05){
//                finishFlag = true;
//            }else{
//                    qDebug()<<"*****************  【 stage  "<<stage<<" is not finished 】*****************";
//            }
//        }else{
//             enabledetect = false;
//             isdetected = false;
//             ActionIndex.store(-2);
//             m_Robot->setLinkHalt();
//        }
//    }

//    //运动到位标志符检测
//    if(finishFlag == true){
//         qDebug()<<"----finishFlag---- : "<<  stage;
//         enabledetect = false;
//         isdetected = false;
//         ActionIndex.store(-2);
//         m_Robot->setLinkHalt();
//         qDebug()<<" ----------------------doaction finished---------------------------";
//         return true;
//    }
//    return false;
//}

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
        if(laserDistance[i] < 300) {
            log->error("{}激光数据有误,或壁面距离太远",i);
            return -1;
        }
    }

    //计算激光距离最大偏差
    double maxDistance = 1000;
    double minDistance = 0;
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
    if(maxDistance - minDistance< PARRALLE_DISTANCE)  //最大偏差小于阈值
    {
        log->info("激光距离最大偏差小于PARRALLE_DISTANCE，完成调平");
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
----------*0*--------------------*1*------------------
------------------------------------------------------
---*5*---------------------------------*2*------------
------------------------------------------------------
----------*4*---------------------*3*-----------------
------------------------------------------------------
 */
int CTask::CheckPositionState()
{
    //视觉检测结果有效性检测
    if(m_stMeasuredata.m_bLineDistance[0]||m_stMeasuredata.m_bLineDistance[4]  == false
    || m_stMeasuredata.m_bLineDistance[1]||m_stMeasuredata.m_bLineDistance[3]  == false
    || m_stMeasuredata.m_bLineDistance[2]||m_stMeasuredata.m_bLineDistance[5] == false)
    {
        log->error("检测到的边线数据不满足调整需求");
        return -1;
    }

    double Ref_Distance = 15;

    //边线偏差是否小于阈值
    double line_dis_1 = m_stMeasuredata.m_bLineDistance[0]*m_stMeasuredata.m_bLineDistance[0]+ m_stMeasuredata.m_bLineDistance[4]*m_stMeasuredata.m_bLineDistance[4] - Ref_Distance;
    double line_dis_2 = m_stMeasuredata.m_bLineDistance[1]*m_stMeasuredata.m_bLineDistance[1]+ m_stMeasuredata.m_bLineDistance[3]*m_stMeasuredata.m_bLineDistance[3]- Ref_Distance;
    double line_dis_3 = m_stMeasuredata.m_bLineDistance[2]*m_stMeasuredata.m_bLineDistance[2]+ m_stMeasuredata.m_bLineDistance[5]*m_stMeasuredata.m_bLineDistance[5] - Ref_Distance;

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