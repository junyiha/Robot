#include "robot.h"


CRobot::CRobot(ComInterface* comm,QObject *parent)
    : QThread{parent}
{
    //修改827
    m_Index = 0; //link索引
    m_Freedom = 9;
    m_ToolFreedom = 2;


    //运动学参数
    double D4 = 930; //腰俯仰轴线到地面距离（d1 =0时）
    double D7 = 1060; //俯仰均为0且伸缩为0时，俯仰轴线到Z轴的距离
    double L5 = 400; //两个俯仰轴的距离
    double L6 = 250;// 大臂俯仰轴到筒轴线距离


    //关节位置参数
    double d1 = 0;
    double d2 = 0;
    double d3 = 0;
    double th4 = 0;
    double th5 = 0;
    double th6 = 0;
    double d7  = 0;
    double th8 = 0;
    double th9 = 0;
    double dt = 0;

    std::vector<CTransformation> DH_List = {
            //DH 参数          α                    A                        θ             D    轴类型) 所有角度均为弧度制！
        CTransformation(  0 * M_PI / 180,	  0,	        -90 * M_PI / 180,	  d1,		Prismatic), //1升降z
        CTransformation(-90 * M_PI / 180,	  0,	        -90 * M_PI / 180,   d2,		Prismatic), //2前后平移
        CTransformation( 90 * M_PI / 180,	  0,	         90 * M_PI / 180,	  d3,		Prismatic), //3左右平移
        CTransformation( 90 * M_PI / 180,	  0,	        th4 * M_PI / 180,	  D4,		Revolute ), //4腰旋转
        CTransformation( 90 * M_PI / 180,   0,	        th5 * M_PI / 180,	   0,	    Revolute ), //5腰俯仰（-15--+15）
        CTransformation(  0 * M_PI / 180,  L5,	        th6 * M_PI / 180,	   0,		Revolute ), //6臂俯仰 (角度不要为0)
        CTransformation( 90 * M_PI / 180,  L6,	          0 * M_PI / 180,	  d7,		Prismatic), //7伸缩 (角度不要为0)
        CTransformation(  0 * M_PI / 180,   0,	        th8 * M_PI / 180,	   0,		Revolute ), //8回转 (角度不要为0)
        CTransformation( -90 * M_PI / 180,   0,	        th9 * M_PI / 180,	   0,		Revolute ), //9腕 (角度不要为0)
    };


    //    DH_Parameter Tool_Mat = DH_Parameter(90 * M_PI / 180, 0, 90 * M_PI / 180, L6, Prismatic);//竖板旋转用，工具坐标系旋转90
    CTransformation Tool_Mat = CTransformation(90* M_PI / 180, 0,  180 * M_PI / 180, dt, Prismatic);       //装横板用，末端工具（可伸缩，但是默认不动，不需要刷新）
    m_LinkModel =new CRobotKinectModel(DH_List,Tool_Mat);



    m_Comm = comm;
    m_running = false;

    std::memset(&m_LinkCmd,0,sizeof(m_LinkCmd));
    std::memset(&_LinkCmd,0,sizeof(_LinkCmd));
    std::memset(&m_AxisCmd,0,sizeof(m_AxisCmd));
    std::memset(&_AxisCmd,0,sizeof(_AxisCmd));

    std::memset(&m_LinkSta,0,sizeof(m_LinkSta));
    std::memset(&_LinkSta,0,sizeof(_LinkSta));

    Eigen::VectorXd endpos= m_LinkModel->getEndPosition();
    std::memcpy(m_ActPose, endpos.data(), 6*sizeof(double));
    std::memset(m_ActJoints,0, 20*sizeof(double));
        st_ReadAxis temp;
        memset(&temp,0,sizeof(temp));
        for(int i=0;i<m_Freedom+1;i++)
        {
            m_JointGroupStatus.push_back(temp);
            _JointGroupStatus.push_back(temp);
        }

    //_EndPose = Eigen::Matrix4d::

    log = spdlog::get("logger");

    //设置关节限制参数
    for(int i = 0;i<m_Freedom;i++)
    {
        if(DH_List[i].Get_Type() == Prismatic)
        {
            m_JointVelLimt[i] = LINK_0_JOINT_MAX_VEL[i];
            m_JointPosLimt[i] = LINK_0_JOINT_LIMIT_POS[i];
            m_JointNegLimt[i] = LINK_0_JOINT_LIMIT_NEG[i];
        }
        else
        {
            m_JointVelLimt[i] = LINK_0_JOINT_MAX_VEL[i]/57.2957795;
            m_JointPosLimt[i] = LINK_0_JOINT_LIMIT_POS[i]/57.2957795;
            m_JointNegLimt[i] = LINK_0_JOINT_LIMIT_NEG[i]/57.2957795;
        }
    }
    m_JointVelLimt[5] = LINK_0_JOINT_MAX_VEL[5];
    m_JointPosLimt[5] = LINK_0_JOINT_LIMIT_POS[5];
    m_JointNegLimt[5] = LINK_0_JOINT_LIMIT_NEG[5];
    //工具关节限制参数
    m_JointVelLimt[m_Freedom] = LINK_0_JOINT_MAX_VEL[3];
    m_JointPosLimt[m_Freedom] = LINK_0_JOINT_LIMIT_POS[3];
    m_JointNegLimt[m_Freedom] = LINK_0_JOINT_LIMIT_NEG[3];
    //修改827 end
}

CRobot::~CRobot()
{
    delete m_LinkModel;
}

void CRobot::run()
{
    log->info("机器人启动运行...");
    m_running = true;
    while(m_running == true)
    {
        //qDebug()<<"机器人运行中------------------------------------------";
        UpdateStatus();//更新状态
        StateMachine();//状态机
        QThread::msleep(50);
    }
    //停止设备
}

void CRobot::setLinkCom(stLinkCommand  cmd)
{
    mutex_sta.lock();

    _LinkCmd = cmd;

    mutex_sta.unlock();
}

stLinkStatus CRobot::getLinkSta()
{
    stLinkStatus sta;

    mutex_sta.lock();

    sta = _LinkSta;

    mutex_sta.unlock();

    return sta;
}

QVector<st_ReadAxis> CRobot::getJointGroupSta()
{
    return m_Comm->getJointGroupStatus();
}

void CRobot::UpdateStatus()
{

    //通过接口获取反馈数据
    QVector<st_ReadAxis> m_JointGroupStatus = m_Comm->getLinkJointStatus(m_Index); //link轴组数据
    stLinkStatus sta = m_Comm->getLinkStatus(m_Index);//link状态数据

    //获取末端工具尺寸
    //修改827
    double len = m_JointGroupStatus[m_Freedom].Position;

    //单位换算degree => rad
    m_JointGroupStatus[3].Position *= 1.0/57.3;
    m_JointGroupStatus[4].Position *= 1.0/57.3;
    m_JointGroupStatus[7].Position *= 1.0/57.3;
    m_JointGroupStatus[8].Position *= 1.0/57.3;


    //修改827
    //更新运动模型数据
    if(m_JointGroupStatus.size() != m_Freedom + m_ToolFreedom) //机器人轴+工具轴
    {
        log->error(" Statedata of axis is unmatch!");
        return;
    }

    std::vector<std::pair<double,bool>> vJointValue;
    for(int i=0;i<m_Freedom; i++)
    {
        m_ActJoints[i] = m_JointGroupStatus[i].Position;
        vJointValue.push_back(std::make_pair(m_ActJoints[i], true));

    }
    vJointValue[4].second = false; // 腰俯仰不参与联动

    m_ActJoints[m_Freedom] = m_JointGroupStatus[m_Freedom].Position;

    m_LinkModel->setToolPos(len, 0);//根据工装实际安装位置确定角度。弧度
    m_LinkModel->setJointPos(vJointValue);
    //修改827 end

    //更新link状态数据
    m_LinkSta.eLinkActState = sta.eLinkActState;
    //qDebug()<<"eLinkActState"<<m_LinkSta.eLinkActState;
    Eigen::VectorXd endpos= m_LinkModel->getEndPosition();   //末端姿态
    std::memcpy(m_ActPose, endpos.data(), 6*sizeof(double));
    std::memcpy(m_LinkSta.stLinkActKin.LinkPos,m_ActPose,6*sizeof(double));


    mutex_sta.lock();
    _LinkSta= m_LinkSta;
    _JointGroupStatus = m_JointGroupStatus;

    mutex_sta.unlock();

    mutex_cmd.lock();
    m_LinkCmd = _LinkCmd;
    std::memset(&_LinkCmd,0,sizeof(_LinkCmd));
    mutex_cmd.unlock();

    mutex_posDev.lock();
    _EndPose = m_LinkModel->getEndPose();
    mutex_posDev.unlock();

}

void CRobot::StateMachine()
{
    //捕获状态改变
    static E_LinkCommd pre_eLinkCommd = eLINK_NONE;
    if(pre_eLinkCommd != m_LinkCmd.eLinkCommd){
       log->info(("eLinkCommd has changed from [" + GetEnumName_LinkCommd(pre_eLinkCommd) + "]------->[" + GetEnumName_LinkCommd(m_LinkCmd.eLinkCommd)+ "]").c_str());
    }
    pre_eLinkCommd = m_LinkCmd.eLinkCommd;


    static E_LinkState pre_eLinkActState = eLINK_STANDSTILL;
    if(pre_eLinkActState != m_LinkSta.eLinkActState){
       log->info(("m_LinkSta.eLinkActState has changed  [" + GetEnumName_LinkState(pre_eLinkActState)+ "]------------>[" + GetEnumName_LinkState(m_LinkSta.eLinkActState)+"]").c_str());
    }
    pre_eLinkActState = m_LinkSta.eLinkActState;

    switch(m_LinkSta.eLinkActState)
    {
    case eLINK_ERRORSTOP:
        switch(m_LinkCmd.eLinkCommd)
        {
        case eLINK_NONE:
            break;
        case  eLINK_RESET:
            m_Comm->LinkReset(m_Index);
            break;
        case  eLINK_HALT:
            m_Comm->LinkHalt(m_Index);
            break;
        case  eLINK_STOP:
            m_Comm->LinkStop(m_Index);
            break;
        case  eLINK_HOME://将当前位置记为零点
        case  eLINK_POWER:

        case  eLINK_MOV:
            qDebug()<<"Error State! Cannot execute command except reset.";
            break;
        default:
            qDebug()<<"Unknown command..";
            break;
        }
        break;
    case eLINK_DISABLED:
        switch(m_LinkCmd.eLinkCommd)
        {
        case eLINK_NONE:
            break;
        case  eLINK_RESET:
            m_Comm->LinkReset(m_Index);
            break;
        case eLINK_HOME:
            //将当前位置记为零点,仅disable状态下可做
            m_Comm->LinkHome(m_Index);
            break;
        case  eLINK_POWER:
            m_Comm->LinkPower(m_Index,m_LinkCmd.stLinkKinPar.bEnable);
            break;
        case  eLINK_STOP:
        case  eLINK_HALT:
        case  eLINK_MOV:
            qDebug()<<"Disable state， cannot execute motion command";
            break;
        default:
            qDebug()<<"Unknown command..";
            break;
        }
        break;
    case eLINK_STANDSTILL:
        switch(m_LinkCmd.eLinkCommd)
        {
        case eLINK_NONE:
            break;
        case  eLINK_RESET:
            m_Comm->LinkReset(m_Index);
            break;
        case eLINK_HOME:
            qDebug()<<"Home command can Only be executed in disable state.";
            break;
        case  eLINK_POWER:
            m_Comm->LinkPower(m_Index,m_LinkCmd.stLinkKinPar.bEnable);
            this->log->info("********Robot m_Index {}, Enable {}*************", m_Index,m_LinkCmd.stLinkKinPar.bEnable);
            break;
        case  eLINK_STOP:
            m_Comm->LinkStop(m_Index);
            break;
        case  eLINK_HALT:
            m_Comm->LinkHalt(m_Index);
            break;
        case  eLINK_MOV:
            StateMachineMove();
            break;
        default:
            qDebug()<<"Unknown command..";
            break;
        }
        break;
    case eLINK_STOPPING:
        switch(m_LinkCmd.eLinkCommd)
        {
        case eLINK_NONE:
            break;
        case  eLINK_RESET:
        case eLINK_HOME:
        case  eLINK_POWER:
        case  eLINK_STOP:
        case  eLINK_HALT:
        case  eLINK_MOV:
            qDebug()<<"Robot is stopping, donnot respond to any command.";
            break;
        default:
            qDebug()<<"Unknown command..";
            break;
        }
        break;

    case eLINK_HOMING:
        switch(m_LinkCmd.eLinkCommd)
        {
        case eLINK_NONE:
            break;
        case  eLINK_RESET:
        case eLINK_HOME:
        case  eLINK_POWER:
        case  eLINK_HALT:
        case  eLINK_MOV:
            qDebug()<<"Robot is stopping, donnot respond to any command except stopping.";
            break;
        case  eLINK_STOP:
            m_Comm->LinkStop(m_Index);
            break;
        default:
            qDebug()<<"Unknown command..";
            break;
        }
        break;
    case eLINK_MOVING:
        switch(m_LinkCmd.eLinkCommd)
        {
        case eLINK_NONE:
            break;
        case  eLINK_RESET:

            break;
        case eLINK_HOME:
            //将当前位置记为零点
            qDebug()<<"Moving state, cannot home,robot is stopped.";
            m_Comm->LinkStop(m_Index);

        case  eLINK_POWER:
            qDebug()<<"Moving state, cannot power,robot is stopped.";
            m_Comm->LinkStop(m_Index);
        case  eLINK_STOP:
            m_Comm->LinkStop(m_Index);
            break;
        case  eLINK_HALT:
            m_Comm->LinkHalt(m_Index);
            break;
        case  eLINK_MOV:
            StateMachineMove();
            break;
        default:
            qDebug()<<"Unknown command..";
            break;
        }
        break;
    default:
        m_Comm->JointStop(m_Index);
        log->warn("unknown link status");
        break;
    }

}


void CRobot::StateMachineMove()
{
    static Eigen:: Matrix4d tar,result;
    Eigen::Map<Eigen::VectorXd> car_pos(m_LinkCmd.stLinkKinPar.LinkPos,6);
    Eigen::Map<Eigen::VectorXd> car_vel(m_LinkCmd.stLinkKinPar.LinkVel,6);

    Eigen::Map<Eigen::VectorXd> joint_pos(m_LinkCmd.stLinkKinPar.LinkPos,m_Freedom+1);
    Eigen::Map<Eigen::VectorXd> joint_vel(m_LinkCmd.stLinkKinPar.LinkVel,m_Freedom+1);

    log->info(("m_LinkCmd.stLinkKinPar.eActMotionMode: " + std::to_string(m_LinkCmd.stLinkKinPar.eActMotionMode)).c_str());


    switch(m_LinkCmd.stLinkKinPar.eActMotionMode)
    {
    case eMotionLineAbsolute:
        //log->info("robot move to position: {}, {}, {}, {}, {}, {}",car_pos(0),car_pos(1),car_pos(2),car_pos(3)*57.3,car_pos(4)*57.3,car_pos(5)*57.3);
        MoveLineAbsBase(car_pos, car_vel);
        break;
    case eMotionLineVelocity:
        log->info("robot move velocity: {}, {}, {}, {}, {}, {}",car_vel(0),car_vel(1),car_vel(2),car_vel(3),car_vel(4),car_vel(5));
        MoveLineVelTool(car_vel);
        break;
    case eMotionMode1: //关节按给定速度运动
        MoveJointVel(joint_pos);
        break;
    case eMotionMode2: //关节移动到目标位置
        MoveJointAbs(joint_pos,joint_vel);
        break;
    default:
        qDebug()<<"unknown link motion mode " <<m_LinkCmd.stLinkKinPar.eActMotionMode;
        m_Comm->LinkHalt(m_Index);
        break;
    }

}

//tarpose:
void CRobot::MoveLineAbsBase(Eigen::Matrix4d tarpose, double vel_max[])
{
    m_LinkSta.stLinkActKin.bDone = false;
    m_LinkSta.stLinkActKin.bBusy = true;
//    qDebug()<<"MoveLineAbsBase****************************************************";


    /*  计算关节速度--基于基坐标****************************************************/
    Eigen::VectorXd end_vel =  m_LinkModel->getTarSpeed(tarpose);   //末端速度


    //偏差值小于位姿分辨率，返回完成
    if(end_vel.head(3).norm() < POSITION_RESOLUTION && end_vel.tail(3).norm() <ROTATE_RESOLUTION )
    {
        m_LinkSta.stLinkActKin.bBusy = false;
        m_LinkSta.stLinkActKin.bDone = true;
        m_Comm->LinkHalt(m_Index);

        return;
    }


    double maxc = 1.0;
    for(int i =0;i<6;i++)
    {
        if(fabs(vel_max[i])<0.0001)
        {
            qDebug()<<"关节"<<i<<"限速过低"<<vel_max[i];
            m_LinkSta.stLinkActKin.bDone = false;
            m_LinkSta.stLinkActKin.bBusy = false;
            m_Comm->LinkHalt(m_Index);
            return;
        }
        maxc = fabs(end_vel[i]/vel_max[i])>maxc ? fabs(end_vel[i]/vel_max[i]):maxc;
    }
    Eigen::VectorXd end_vel_limit = end_vel/maxc; //末端速度：根据限速等比例缩放位姿偏差

    Eigen::VectorXd joint_vel = m_LinkModel->getJointVel(end_vel_limit,false);//关节速度



    double maxj = 1.0;
    for(int i=0;i<m_Freedom;i++)
    {
        if(m_JointVelLimt[i]<0.01)
        {
            qDebug()<<"关节"<<i<<"限速过低"<<m_JointVelLimt[i];
            m_LinkSta.stLinkActKin.bDone = false;
            m_LinkSta.stLinkActKin.bBusy = false;
            m_Comm->LinkHalt(m_Index);
            return;
        }
        maxj = fabs(joint_vel[i]/m_JointVelLimt[i])>maxj?fabs(joint_vel[i]/m_JointVelLimt[i]):maxj;
    }
    Eigen::VectorXd joint_vel_limt = joint_vel/maxj; //限速后的关节速度

    //qDebug()<<"joint_vel"<<joint_vel[0]<<","<<joint_vel[1]<<","<<joint_vel[2];

    /*  择执行运动****************************************************
    */

    double joint_vel_set[20];       //数组类型：限速后的关节速度
    double joint_pos[20];           //微动时，目标关节位置
    for(int i=0;i<m_Freedom;i++)
    {
        joint_vel_set[i] = joint_vel_limt[i];
        joint_pos[i] = joint_vel[i]+m_ActJoints[i];

    }

    //单位转化->下发速度
    joint_pos[3] *= 57.2957795;
    joint_pos[4] *= 57.2957795;
    joint_pos[5] *= 57.2957795;

    joint_vel_set[3] *= 57.2957795;
    joint_vel_set[4] *= 57.2957795;
    joint_vel_set[5] *= 57.2957795;

    Eigen::VectorXd joint_vel_degree(joint_vel.size());
    for(int i=0;i<joint_vel.size();i++)
    {
        joint_vel_degree[i]=joint_vel_set[i];

    }
    //工具保持动

    m_Comm->setLinkJointMoveAbs(m_Index,joint_pos,joint_vel_set);

    m_LinkSta.stLinkActKin.bDone = false;
    m_LinkSta.stLinkActKin.bBusy = true;
}



void CRobot::MoveLineAbsBase(Eigen::VectorXd car_pos, Eigen::VectorXd vel_max)
{    
    m_LinkSta.stLinkActKin.bDone = false;
    m_LinkSta.stLinkActKin.bBusy = true;

     /*  输入合法性检查****************************************************/
    if(car_pos.size() != 6 || vel_max.size() != 6)
    {
        log->error("the size of car_pos  and vel_max must be 6.");
        m_Comm->LinkHalt(m_Index);
        log->info("robot has been halt");
        return;
    }

    /*  计算是否已靠近目标****************************************************/
    Eigen::Vector3d dposition(3);
    Eigen::Vector3d dposture(3);
    for(int i=0;i<3;i++)
    {
        dposition[i] =  car_pos[i] - m_ActPose[i];
        dposture[i] = car_pos[i+3] - m_ActPose[i+3];
    }
    double len_pos = dposition.norm(); //位置偏差模
    double len_angle = dposture.norm();//姿态偏差模

    //偏差值小于位姿分辨率，返回完成
    if(len_pos < POSITION_RESOLUTION && len_angle<ROTATE_RESOLUTION)
    {
        m_LinkSta.stLinkActKin.bBusy = false;
        m_LinkSta.stLinkActKin.bDone = true;
        m_Comm->LinkHalt(m_Index);
        return;
    }

    /*  根据限速计算末端移动矢量****************************************************/

    //末端位移矢量
    Eigen::VectorXd end_pos_delta =  m_LinkModel->getTarSpeed(car_pos);


    //根据 末端限速+末端位移矢量 =>计算速度矢量
    double maxc = 1;
    for(int i =0;i<6;i++)
    {
        if(fabs(vel_max[i])<0.0001)
        {
            log->warn("末端速度项{}过低：vel_max = {}",i,vel_max[i]);
            m_LinkSta.stLinkActKin.bDone = true;
            m_LinkSta.stLinkActKin.bBusy = false;
            m_Comm->LinkHalt(m_Index);
            return;
        }
        maxc = fabs(end_pos_delta[i]/vel_max[i])>maxc ? fabs(end_pos_delta[i]/vel_max[i]):maxc;
    }
    Eigen::VectorXd vel_end = end_pos_delta/maxc; //末端速度矢量



    /*  计算关节速度--基于基坐标****************************************************/

    //根据 雅克比计算关节速度
    Eigen::VectorXd joint_vel = m_LinkModel->getJointVel(vel_end,false);

    //根据关节限速计算关节输出速度
    double maxj = 1.0;
    for(int i=0;i<m_Freedom;i++)
    {
        if(m_JointVelLimt[i]<0.01)
        {
            qDebug()<<"关节"<<i<<"限速过低"<<m_JointVelLimt[i];
            m_LinkSta.stLinkActKin.bDone = false;
            m_LinkSta.stLinkActKin.bBusy = false;
            m_Comm->LinkHalt(m_Index);
            return;
        }
        maxj = fabs(joint_vel[i]/m_JointVelLimt[i])>maxj?fabs(joint_vel[i]/m_JointVelLimt[i]):maxj;
    }

    //限速后的关节速度
    Eigen::VectorXd joint_vel_limt = joint_vel/maxj;

    //关节目标位置
    double joint_pos[20];           //微动时，目标关节位置
    double joint_vel_set[20];       //转换为下发函数数据类型
    for(int i=0;i<m_Freedom;i++)
    {
        joint_pos[i] = joint_vel[i]+m_ActJoints[i];
        joint_vel_set[i] = joint_vel_limt[i];
    }

    /*  下发关节速度指令****************************************************
    */

    //角度单位：弧度->deg
    joint_pos[3] *= 57.2957795;
    joint_pos[4] *= 57.2957795;
    joint_pos[7] *= 57.2957795;
    joint_pos[8] *= 57.2957795;

 

     //单位换算rad->degree
    joint_vel_set[3] *= 57.2957795;
    joint_vel_set[4] *= 57.2957795;
    joint_vel_set[7] *= 57.2957795;
    joint_vel_set[8] *= 57.2957795;

 
    //工具轴保持动
    joint_pos[9]     = m_ActJoints[9];
    joint_vel_set[9] = 0;

    //下发指令到设备
    m_Comm->setLinkJointMoveAbs(m_Index,joint_pos,joint_vel_set);

    //qDebug()<<"下发位置:"<<joint_pos[0]<<" "<<joint_pos[1]<<" "<<joint_pos[2]<<" "<<joint_pos[3]<<" "<<joint_pos[4]<<" "<<joint_pos[5];
    //qDebug()<<"下发速度:"<<joint_vel_set[0]<<" "<<joint_vel_set[1]<<" "<<joint_vel_set[2]<<" "<<joint_vel_set[3]<<" "<<joint_vel_set[4]<<" "<<joint_vel_set[5];

    m_LinkSta.stLinkActKin.bDone = false;
    m_LinkSta.stLinkActKin.bBusy = true;
}

void CRobot::MoveLineVelTool(Eigen::VectorXd car_vel)
{
    m_LinkSta.stLinkActKin.bDone = false;


    /*  输入合法性检查****************************************************/
   if(car_vel.size() != 6 )
   {
       log->error("the size of car_vel must be 6.");
       m_Comm->LinkHalt(m_Index);
       log->info("robot has been halt");
       return;
   }
   m_LinkSta.stLinkActKin.bBusy = true;

    /*  计算关节速度-基于末端坐标****************************************************/
    Eigen::VectorXd joint_vel = m_LinkModel->getJointVel(car_vel,true);
    double maxj = 1.0;
    for(int i=0;i<m_Freedom;i++)
    {
        //qDebug()<<i<<":"<<joint_vel[i];

        if(m_JointVelLimt[i]<0.01)
        {
            qDebug()<<"关节"<<i<<"限速过低"<<m_JointVelLimt[i];
            m_LinkSta.stLinkActKin.bDone = false;
            m_LinkSta.stLinkActKin.bBusy = false;
            m_Comm->LinkHalt(m_Index);
            return;
        }
        maxj = fabs(joint_vel[i]/m_JointVelLimt[i])>maxj?fabs(joint_vel[i]/m_JointVelLimt[i]):maxj;
        //qDebug()<<i<<":"<<m_JointVelLimt[i];
    }

    //qDebug()<<"maxj:"<<maxj;
    double joint_vel_set[20];
    for(int i=0;i<m_Freedom;i++)
    {
        joint_vel_set[i] = joint_vel[i]/maxj;


    }

    //单位换算rad->degree
    joint_vel_set[3] *= 57.2957795;
    joint_vel_set[4] *= 57.2957795;
    joint_vel_set[7] *= 57.2957795;
    joint_vel_set[8] *= 57.2957795;

    //工具轴保持动
    joint_vel_set[9] = 0;


    m_Comm->setLinkJointMoveVel(m_Index,joint_vel_set);
    log->info("MoveLineVelTool joint vel:{},{},{},{},{},{},{},{},{},{}",joint_vel_set[0],joint_vel_set[1],joint_vel_set[2],joint_vel_set[3],joint_vel_set[4],joint_vel_set[5],joint_vel_set[6],joint_vel_set[7],joint_vel_set[8],joint_vel_set[9]);


    m_LinkSta.stLinkActKin.bDone = false;
    m_LinkSta.stLinkActKin.bBusy = true;

}


void CRobot::MoveJointVel(Eigen::VectorXd joint_vel)
{
    m_LinkSta.stLinkActKin.bDone = false;
    m_LinkSta.stLinkActKin.bBusy = true;

    double maxj = 1.0;
    for(int i=0;i<m_Freedom;i++)
    {
        if(m_JointVelLimt[i]<0.1)
        {
            qDebug()<<"关节"<<i<<"限速过低"<<m_JointVelLimt[i];
            m_LinkSta.stLinkActKin.bDone = false;
            m_LinkSta.stLinkActKin.bBusy = false;
            m_Comm->LinkHalt(m_Index);
            return;
        }
        maxj = fabs(joint_vel[i]/m_JointVelLimt[i])>maxj?fabs(joint_vel[i]/m_JointVelLimt[i]):maxj;
    }


    double joint_vel_set[20];//限速后的关节速度
    for(int i=0;i<m_Freedom;i++) //根据关节限速，等比例缩放所有轴速度
    {
        joint_vel_set[i] = joint_vel[i]/maxj;
    }

    m_Comm->setLinkJointMoveVel(m_Index,joint_vel_set);
    m_LinkSta.stLinkActKin.bDone = false;
    m_LinkSta.stLinkActKin.bBusy = true;
}


void CRobot::MoveJointAbs(Eigen::VectorXd joint_pos,Eigen::VectorXd joint_vel)
{
    m_LinkSta.stLinkActKin.bDone = false;
    m_LinkSta.stLinkActKin.bBusy = true;


    Eigen::VectorXd joint_relative;  //关节移动角度
    for(int i=0;i<m_Freedom;i++)
    {
        joint_relative[i] = joint_pos[i] - m_ActJoints[i];
    }

    //计算移动速度，约束：同步到达、设定速度、速度限制
    double maxj = 1.0;
    for(int i=0;i<m_Freedom;i++)
    {
        if(m_JointVelLimt[i]<0.1)
        {
            qDebug()<<"关节"<<i<<"限速过低"<<m_JointVelLimt[i];
             m_LinkSta.stLinkActKin.bDone = false;
            m_LinkSta.stLinkActKin.bBusy = false;
            m_Comm->LinkHalt(m_Index);
            return;
        }
        maxj = fabs(joint_vel[i]/m_JointVelLimt[i])>maxj?fabs(joint_vel[i]/m_JointVelLimt[i]):maxj;
    }
    Eigen::VectorXd joint_vel_limt; //限速后的关节速度
    for(int i=0;i<m_Freedom;i++)
    {
        joint_vel_limt[i] = joint_vel[i]/maxj;
    }

    //计算限速下各轴移动时间，选取最大值
    double maxtime = 0.1;
    for(int i=0;i<m_Freedom;i++)
    {
        if(joint_vel_limt[i]<0.1)
        {
            qDebug()<<"关节"<<i<<"限速过低"<<joint_vel_limt[i];
             m_LinkSta.stLinkActKin.bDone = false;
            m_LinkSta.stLinkActKin.bBusy = false;
            m_Comm->LinkHalt(m_Index);
            return;
        }
        maxtime = fabs(joint_relative[i]/joint_vel_limt[i])>maxtime?fabs(joint_relative[i]/joint_vel_limt[i]):maxtime;
    }
    //根据关节最大运动时间，计算所有轴速度
    double joint_vel_set[20];
    double joint_pos_set[20];
    for(int i=0;i<m_Freedom;i++)
    {
        joint_vel_set[i] = joint_relative[i]/maxtime;
        joint_pos_set[i] = joint_pos[i];
    }

    double distance = joint_relative.norm(); //移动距离的模
    if(distance < JOINT_VEL_MOVE)
    {
        m_Comm->setLinkJointMoveAbs(m_Index,joint_pos_set,joint_vel_set);

    }else           //位置偏差较大时采用速度模式
    {
        m_Comm->setLinkJointMoveVel(m_Index,joint_vel_set);
    }

}

void CRobot::setLinkEnable(bool enable)
{
    mutex_cmd.lock();

    _LinkCmd.eLinkCommd = eLINK_POWER;
    _LinkCmd.stLinkKinPar.bEnable = enable;

    mutex_cmd.unlock();
}

void CRobot::setLinkReset()
{
    mutex_cmd.lock();

    _LinkCmd.eLinkCommd = eLINK_RESET;

    mutex_cmd.unlock();
}

void CRobot::setLinkHalt()
{
    mutex_cmd.lock();

    _LinkCmd.eLinkCommd = eLINK_HALT;

    mutex_cmd.unlock();
}

void CRobot::setLinkStop()
{
    mutex_cmd.lock();

    _LinkCmd.eLinkCommd = eLINK_STOP;

    mutex_cmd.unlock();
}

void CRobot::setJointMoveVel(uint index, double vel)
{
    m_Comm->JointMove(index,eMC_MOV_VEL,0,vel);
}

void CRobot::setJointMoveAbs(uint index, double pos, double vel)
{
   m_Comm->JointMove(index,eMC_MOV_ABS,pos,vel);
}

void CRobot::setJointGroupMoveVel(double vel[])
{
    double pos[20] = {0};
    m_Comm->setJointGroupMove(eMC_MOV_VEL,pos,vel);
}

void CRobot::setJointGroupMoveAbs( const double pos[], const double vel[])
{
   m_Comm->setJointGroupMove(eMC_MOV_ABS,pos,vel);
}

void CRobot::setLinkMoveVel(double vel[])
{
    mutex_cmd.lock();

    _LinkCmd.eLinkCommd = eLINK_MOV;
    _LinkCmd.stLinkKinPar.eActMotionMode = eMotionLineVelocity ;
    for(int i=0 ;i<6; ++i){
        _LinkCmd.stLinkKinPar.LinkVel[i] = vel[i];
    }

    mutex_cmd.unlock();
}

void CRobot::setLinkMoveAbs(const double pos[], const double vel[])
{
    mutex_cmd.lock();

    _LinkCmd.eLinkCommd = eLINK_MOV;
    _LinkCmd.stLinkKinPar.eActMotionMode = eMotionLineAbsolute;
//    _LinkCmd.stResverInput.lReserve[0] = step;
    for(int i=0;i<6;++i){
        _LinkCmd.stLinkKinPar.LinkPos[i] = pos[i];
        _LinkCmd.stLinkKinPar.LinkVel[i] = vel[i];
    }

    mutex_cmd.unlock();
}



QVector<double> CRobot::getTargetPose(const Eigen::Matrix4d tar_rt)
{

    //目标位姿矩阵
    mutex_posDev.lock();
    Eigen::Matrix4d tar_pose = _EndPose *tar_rt;
    mutex_posDev.unlock();

    Eigen::Matrix3d Rotate;
    for(int i = 0;i<3;i++)
    {
        for(int j = 0;j<3;j++)
        {
            Rotate(i,j) = tar_pose(i,j);
        }
    }
    //计算目标位姿
    double RY = atan2( Rotate(0,2),sqrt(Rotate(1,2) * Rotate(1,2) + Rotate(2,2)*Rotate(2,2) ));
    if(fabs(RY-M_PI/2) < 0.0001){
        RY = M_PI/2 - 0.0001;
    }

    double RX = atan2( -Rotate(1,2)/cos(RY),Rotate(2,2)/cos(RY) );
    double RZ = atan2( -Rotate(0,1)/cos(RY),Rotate(0,0)/cos(RY) );

    QVector<double> pose(6,0);

    pose[0] = tar_pose(0,3);
    pose[1] = tar_pose(1,3);
    pose[2] = tar_pose(2,3);
    pose[3] = RX;
    pose[4] = RY;
    pose[5] = RZ;

    return pose;


}

bool CRobot::isJointReached(QVector<double> tarpos)
{

    mutex_sta.lock();
    QVector<st_ReadAxis>  jointpos = _JointGroupStatus;
    mutex_sta.unlock();

    if(tarpos.size() != m_Freedom+1)
    {
        log->warn("ERROR: Joint num is not equare to link freedom");
        return false;
    }
    for(int i=0;i<m_Freedom+1;i++)
    {
        if(fabs(tarpos[i] - jointpos[i].Position)>0.1)
        {
            return false;
        }
    }

    return true;
}

bool CRobot::isEndReached(QVector<double> tarpos)
{
    mutex_sta.lock();
    stLinkStatus  LinkStatus = _LinkSta;
    mutex_sta.unlock();

    if(tarpos.size() != 6)
    {
        log->warn("ERROR: pose dim must be 6!");
        return false;
    }
//    log->info("pos differ: {},{},{},{},{},{}\n",LinkStatus.stLinkActKin.LinkPos[0] - tarpos[0],
//                                                  LinkStatus.stLinkActKin.LinkPos[1] - tarpos[1],
//                                                  LinkStatus.stLinkActKin.LinkPos[2] - tarpos[2],
//                                                  (LinkStatus.stLinkActKin.LinkPos[3] - tarpos[3]) * 57.3,
//                                                  (LinkStatus.stLinkActKin.LinkPos[4] - tarpos[4]) * 57.3,
//                                                  (LinkStatus.stLinkActKin.LinkPos[5] - tarpos[5]) * 57.3);
    for(int i=0;i<3;i++)
    {
        if(fabs(tarpos[i] - LinkStatus.stLinkActKin.LinkPos[i])>POSITION_RESOLUTION)
        {
            return false;
        }

        if(fabs(tarpos[i+3] - LinkStatus.stLinkActKin.LinkPos[i+3])>ROTATE_RESOLUTION)
        {
            return false;
        }
    }

    return true;
}


void CRobot::setRobotStop()
{
    m_Comm->RobotStop();

}
void CRobot::setRobotHalt()
{
    m_Comm->RobotHalt();
}

void CRobot::setRobotEnable(bool enable)
{
    m_Comm->RobotPower(enable);
}

void CRobot::setRobotReset()
{
    m_Comm->RobotReset();
}

std::string CRobot::GetEnumName_LinkCommd(E_LinkCommd value)
{
    auto it = enumToString_LinkCommd.find(value);
    if (it != enumToString_LinkCommd.end()) {
        return it->second;
    } else {
        throw std::runtime_error("Unknown enum value");
    }
}

std::string CRobot::GetEnumName_LinkState(E_LinkState value)
{
    auto it = enumToString_LinkState.find(value);
    if (it != enumToString_LinkState.end()) {
        return it->second;
    } else {
        throw std::runtime_error("Unknown enum value");
    }
}


void CRobot::closeThread(){
    this->m_running = false;
    QThread::wait();
}


