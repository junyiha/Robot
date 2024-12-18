#include "RobotCom.h"

RobotCom::RobotCom()
{
    for (int i = 0; i < 6; i++)
    {
        if (0 != LINK_FREEDOM[i])
        {
            m_LinkNum++;
            m_JointNum = m_JointNum + LINK_FREEDOM[i];
        }
    }
    m_Hour = 0;
    m_endhour = 0;
    m_SysTime = { 0 };
    m_SendSize = sizeof(DINT) + sizeof(stRobotCommand) + m_LinkNum * sizeof(stLinkCommand) + m_JointNum * sizeof(st_SetAxis);
    m_RecvSize = sizeof(DINT) + sizeof(stRobotStatus) + m_LinkNum * sizeof(stLinkStatus) + m_JointNum * sizeof(st_ReadAxis);
    if (m_SendSize > TCPCOM_DATA_SIZE || m_RecvSize > TCPCOM_DATA_SIZE)
    {
        qDebug() << "Error recv length is bigger than TCPCOM_DATA_SIZE";
    }

    m_UpperHeartbeat = 0;
    memset(&m_RobotCmmd, 0, sizeof(stRobotCommand));
    memset(m_LinkCommd, 0, MAX_FREEDOM_LINK * sizeof(stLinkCommand));
    memset(m_AxisCommd, 0, MAX_FREEDOM_ROBOT * sizeof(st_SetAxis));
    m_RobotHeartbeat = 0;
    memset(&m_RobotStaus, 0, sizeof(stRobotStatus));
    memset(m_LinkStatus, 0, MAX_FREEDOM_LINK * sizeof(stLinkStatus));
    memset(m_AxisStatus, 0, MAX_FREEDOM_ROBOT * sizeof(st_ReadAxis));

    // com连接成功发送信号则打开loop，启动两个定时器；连接断开发送信号关闭loop。
    connect(this, &RobotCom::sigConnected, this, &RobotCom::slotStartLoop);
    connect(this, &RobotCom::sigDisconnected, this, &RobotCom::slotStopLoop);

    // 定时器连接senddata函数
    m_crobotSendTimer = new QTimer(this);
    m_crobotSendTimer->setInterval(50);
    connect(m_crobotSendTimer, &QTimer::timeout, this, &RobotCom::SendData);

    // 定时器连接recvdata函数
    m_crobotRecvTimer = new QTimer(this);
    m_crobotRecvTimer->setInterval(10);
    connect(m_crobotRecvTimer, &QTimer::timeout, this, &RobotCom::RecvData);
}

RobotCom::~RobotCom()
{
}

void RobotCom::slotStartLoop()
{
    m_crobotSendTimer->start();
    m_crobotRecvTimer->start();
}

void RobotCom::slotStopLoop()
{
    m_crobotSendTimer->stop();
    m_crobotRecvTimer->stop();
}

DINT RobotCom::RecvData()
{
    if (m_CommState == true)
    {
        DINT re = Recvbuffer();
        if (0 != re)
        {
            //            qDebug()<<"failed to recv\n";
            return -1;
        }

        mutex_Read.lock();
        {
            ULONG offset = 0;
            memcpy(&m_RobotHeartbeat, m_RecvData + offset, sizeof(DINT));
            offset = offset + sizeof(DINT);
            memcpy(&m_RobotStaus, m_RecvData + offset, sizeof(stRobotStatus));
            offset = offset + sizeof(stRobotStatus);
            int jointcnt = 0;
            for (int i = 0; i < 6; i++)
            {
                if (0 != LINK_FREEDOM[i])
                {
                    memcpy(&(m_LinkStatus[i]), m_RecvData + offset, sizeof(stLinkStatus));
                    offset = offset + sizeof(stLinkStatus);
                    for (int j = 0; j < LINK_FREEDOM[i]; j++)
                    {
                        memcpy(&(m_AxisStatus[jointcnt]), m_RecvData + offset, sizeof(st_ReadAxis));
                        offset = offset + sizeof(st_ReadAxis);
                        jointcnt++;
                    }
                }
            }
        }

        mutex_Read.unlock();

        // emit sigRobotDataUpdate();
        return GetTime() - m_RobotHeartbeat;
    }
}

DINT RobotCom::SendData()
{
    //log->info("**************send data******************");
    if (m_CommState == true)
    {
        m_UpperHeartbeat = GetTime();
        ULONG offset = 0;

        mutex_send.lock();

        memcpy(m_SendData + offset, &m_UpperHeartbeat, sizeof(DINT));
        offset = offset + sizeof(DINT);
        memcpy(m_SendData + offset, &m_RobotCmmd, sizeof(stRobotCommand));
        offset = offset + sizeof(stRobotCommand);
        int jointcnt = 0;
        for (int i = 0; i < 6; i++)
        {
            if (0 != LINK_FREEDOM[i])
            {
                memcpy(m_SendData + offset, &(m_LinkCommd[i]), sizeof(stLinkCommand));
                offset = offset + sizeof(stLinkCommand);
                for (int j = 0; j < LINK_FREEDOM[i]; j++)
                {
                    memcpy(m_SendData + offset, &(m_AxisCommd[jointcnt]), sizeof(st_SetAxis));
                    offset = offset + sizeof(st_SetAxis);
                    jointcnt++;
                }
            }
        }

        m_UpperHeartbeat = 0;
        memset(&m_RobotCmmd, 0, sizeof(stRobotCommand));
        memset(m_LinkCommd, 0, MAX_FREEDOM_LINK * sizeof(stLinkCommand));
        memset(m_AxisCommd, 0, MAX_FREEDOM_ROBOT * sizeof(st_SetAxis));

        mutex_send.unlock();

        // 发送数据
        int re = Sendbuffer();
        if (0 == re)
        {
            return 0;
        }
        else
        {
            qDebug() << "failed to send\n";
            return -1;
        }
    }
    // 标志位没开
    else
        return -10;
}

DINT RobotCom::GetTime()
{

    mutex_Read.lock();
    GetLocalTime(&m_SysTime);
    if (m_SysTime.wMinute > 30)
    {
        m_endhour = 1;
    }
    else
    {
        m_Hour = m_Hour + m_endhour;
        m_endhour = 0;
    }
    DINT acttime = m_Hour * 3600000 + m_SysTime.wMinute * 60000 + m_SysTime.wSecond * 1000 + m_SysTime.wMilliseconds;

    mutex_Read.unlock();

    return acttime;
}

void RobotCom::EndVelMove(uint i, double vel, bool direction, bool underTool)
{
    mutex_send.lock();

    mutex_send.unlock();
}

void RobotCom::EndPosMove(double* dp, bool underTool)
{
    mutex_send.lock();

    mutex_send.unlock();
}

void RobotCom::RobotReset()
{
    mutex_send.lock();

    this->m_RobotCmmd.eCommd = eROBOT_RESET;

    mutex_send.unlock();
}

void RobotCom::RobotPower(bool enable)
{
    mutex_send.lock();

    this->m_RobotCmmd.eCommd = eROBOT_POWER;
    this->m_RobotCmmd.stKinPar.bEnable = enable;

    mutex_send.unlock();
}

void RobotCom::RobotHome()
{
    mutex_send.lock();

    this->m_RobotCmmd.eCommd = eROBOT_HOME;

    mutex_send.unlock();
}

void RobotCom::RobotHalt()
{
    mutex_send.lock();

    this->m_RobotCmmd.eCommd = eROBOT_HALT;

    mutex_send.unlock();
}

void RobotCom::RobotStop()
{
    mutex_send.lock();

    this->m_RobotCmmd.eCommd = eROBOT_STOP;

    mutex_send.unlock();
}

void RobotCom::LinkReset(uint index)
{
    mutex_send.lock();

    this->m_LinkCommd[index].eLinkCommd = eLINK_RESET;

    mutex_send.unlock();
}

void RobotCom::LinkPower(uint index, bool enble)
{
    mutex_send.lock();

    this->m_LinkCommd[index].eLinkCommd = eLINK_POWER;
    this->m_LinkCommd[index].stLinkKinPar.bEnable = enble;

    mutex_send.unlock();
}

void RobotCom::LinkHome(uint index)
{
    mutex_send.lock();

    this->m_LinkCommd[index].eLinkCommd = eLINK_HOME;

    mutex_send.unlock();
}

void RobotCom::LinkHalt(uint index)
{
    mutex_send.lock();

    this->m_LinkCommd[index].eLinkCommd = eLINK_HALT;

    mutex_send.unlock();
}

void RobotCom::LinkStop(uint index)
{
    mutex_send.lock();

    this->m_LinkCommd[index].eLinkCommd = eLINK_STOP;

    mutex_send.unlock();
}

void RobotCom::LinkMove(uint index, E_MotionMode mode, double pos[], double vel[], int freedom)
{
    mutex_send.lock();

    this->m_LinkCommd[index].eLinkCommd = eLINK_MOV;
    this->m_LinkCommd[index].stLinkKinPar.eActMotionMode = mode;
    memcpy(this->m_LinkCommd[index].stLinkKinPar.LinkPos, pos, freedom * 8);
    memcpy(this->m_LinkCommd[index].stLinkKinPar.LinkVel, vel, freedom * 8);

    mutex_send.unlock();
}

void RobotCom::JointPower(uint index, bool enble)
{
    mutex_send.lock();

    this->m_AxisCommd[index].eMC_Motion = eMC_POWER;
    this->m_AxisCommd[index].bEnable = enble;

    mutex_send.unlock();
}

void RobotCom::JointReset(uint index)
{
    mutex_send.lock();

    this->m_AxisCommd[index].eMC_Motion = eMC_RESET;

    mutex_send.unlock();
}

void RobotCom::JointHome(uint index, double pos)
{
    mutex_send.lock();

    this->m_AxisCommd[index].eMC_Motion = eMC_HOME;
    this->m_AxisCommd[index].Position = pos;

    mutex_send.unlock();
}

void RobotCom::JointHalt(uint index)
{
    mutex_send.lock();

    this->m_AxisCommd[index].eMC_Motion = eMC_HALT;

    mutex_send.unlock();
}

void RobotCom::JointStop(uint index)
{
    mutex_send.lock();

    this->m_AxisCommd[index].eMC_Motion = eMC_STOP;

    mutex_send.unlock();
}

bool RobotCom::JointMove(uint index, eMC_Motion mode, double pos, double vel, double end_vel, double distance)
{
    bool re = true;

    mutex_send.lock();

    switch (mode)
    {
    case eMC_MOV_ABS:
        this->m_AxisCommd[index].eMC_Motion = mode;
        this->m_AxisCommd[index].Position = pos;
        this->m_AxisCommd[index].Velocity = vel;
        break;
    case eMC_MOV_CON_ABS:
        this->m_AxisCommd[index].eMC_Motion = mode;
        this->m_AxisCommd[index].Position = pos;
        this->m_AxisCommd[index].Velocity = vel;
        this->m_AxisCommd[index].EndVelocity = end_vel;
        break;
    case eMC_MOV_RELATIVE:
        this->m_AxisCommd[index].eMC_Motion = mode;
        this->m_AxisCommd[index].Distance = distance;
        this->m_AxisCommd[index].Velocity = vel;
        break;
    case eMC_MOV_VEL:
        this->m_AxisCommd[index].eMC_Motion = mode;
        this->m_AxisCommd[index].Velocity = vel;
        break;
    default:
        re = false;
        break;
    }

    mutex_send.unlock();

    return re;
}

bool RobotCom::setJointGroupMove(eMC_Motion mode, const double pos[], const double vel[], double end_vel[], double distance[])
{
    bool re = true;

    mutex_send.lock();
    for (int i = 0; i < m_JointNum; i++)
    {
        switch (mode)
        {
        case eMC_MOV_ABS:
            this->m_AxisCommd[i].eMC_Motion = mode;
            this->m_AxisCommd[i].Position = pos[i];
            this->m_AxisCommd[i].Velocity = vel[i];
            break;
        case eMC_MOV_CON_ABS:
            this->m_AxisCommd[i].eMC_Motion = mode;
            this->m_AxisCommd[i].Position = pos[i];
            this->m_AxisCommd[i].Velocity = vel[i];
            this->m_AxisCommd[i].EndVelocity = end_vel[i];
            break;
        case eMC_MOV_RELATIVE:
            this->m_AxisCommd[i].eMC_Motion = mode;
            this->m_AxisCommd[i].Distance = distance[i];
            this->m_AxisCommd[i].Velocity = vel[i];
            break;
        case eMC_MOV_VEL:
            this->m_AxisCommd[i].eMC_Motion = mode;
            this->m_AxisCommd[i].Velocity = vel[i];
            break;
        default:
            re = false;
            break;
        }
    }
    mutex_send.unlock();
    return re;
}

bool RobotCom::setLinkGroupMove(uint index, eMC_Motion mode, const double pos[], double vel[], double end_vel[], double distance[])
{
    bool re = true;
    int first_joint = 0;
    for (uint i = 0; i < index; i++)
    {
        first_joint += LINK_FREEDOM[i];
    }
    mutex_send.lock();
    for (int i = 0; i < LINK_FREEDOM[index]; i++)
    {
        switch (mode)
        {
        case eMC_MOV_ABS:
            this->m_AxisCommd[i + first_joint].eMC_Motion = mode;
            this->m_AxisCommd[i + first_joint].Position = pos[i];
            this->m_AxisCommd[i + first_joint].Velocity = vel[i];
            break;
        case eMC_MOV_CON_ABS:
            this->m_AxisCommd[i + first_joint].eMC_Motion = mode;
            this->m_AxisCommd[i + first_joint].Position = pos[i];
            this->m_AxisCommd[i + first_joint].Velocity = vel[i];
            this->m_AxisCommd[i + first_joint].EndVelocity = end_vel[i];
            break;
        case eMC_MOV_RELATIVE:
            this->m_AxisCommd[i + first_joint].eMC_Motion = mode;
            this->m_AxisCommd[i + first_joint].Distance = distance[i];
            this->m_AxisCommd[i + first_joint].Velocity = vel[i];
            break;
        case eMC_MOV_VEL:
            this->m_AxisCommd[i + first_joint].eMC_Motion = mode;
            this->m_AxisCommd[i + first_joint].Velocity = vel[i];
            break;
        default:
            re = false;
            break;
        }
    }
    mutex_send.unlock();
    return re;
}

QVector<st_ReadAxis> RobotCom::getLinkJointStatus(uint index)
{

    QVector<st_ReadAxis> re;
    uint joint_index = 0;
    uint freedom = LINK_FREEDOM[index];
    for (uint i = 0; i < index; i++)
    {
        joint_index += LINK_FREEDOM[i];
    }
    mutex_Read.lock();
    for (uint i = joint_index; i < joint_index + freedom; i++)
    {
        re.push_back(m_AxisStatus[i]);
    }
    mutex_Read.unlock();
    return re;
}

StatusofLink RobotCom::getLinkStatus(uint index)
{
    StatusofLink state;
    mutex_Read.lock();
    state = m_LinkStatus[index];
    mutex_Read.unlock();
    return state;
}

QVector<st_ReadAxis> RobotCom::getJointGroupStatus()
{
    QVector<st_ReadAxis> re;

    mutex_Read.lock();
    for (int i = 0; i < m_JointNum; i++)
    {
        re.push_back(m_AxisStatus[i]);
    }
    mutex_Read.unlock();
    return re;
}
