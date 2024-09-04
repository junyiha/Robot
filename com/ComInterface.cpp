//启用本地版本管理---
#include "ComInterface.h"
ComInterface *ComInterface::getInstance()
{
    static ComInterface instance;

    return &instance;
}

ComInterface::ComInterface(QObject *parent) : QThread(parent)
{
    log = spdlog::get("logger");


}

ComInterface::~ComInterface()
{
    running = false;

    m_cRobot.close();
    m_cTools.close();
    m_cManual.close();
    msleep(10000);
    qDebug("destruct commInterface");
}

QVector<double> ComInterface::getLasersDistance()//
{
    return m_cTools.getLaserDistance();

}

QVector<st_ReadAxis> ComInterface::getJointGroupStatus()
{
    return m_cRobot.getJointGroupStatus();
}

void ComInterface::run()
{
    //需要初始化的参数
    const uint DEVICE_NUM = 4;
    uint tryToConnectCount[DEVICE_NUM] = {0} ;
    const uint COUNT_LIMIT = 100;//尝试重连次数上限

    while(this->running)
    {
        //监测机器人连接状态
        if(m_cRobot.getCommState() == false){
            emit m_cRobot.sigDisconnected();
            qDebug()<<"m_cRobot.slotStopLoop();";
            qDebug()<<"尝试连接机器人。。。。"<<tryToConnectCount[0];
            isRobotConnected = m_cRobot.ConnectToServer(g_str_robotip, g_i_robotport);
            if(tryToConnectCount[0] <= COUNT_LIMIT){
                ++tryToConnectCount[0];
            }else{
                log->error("机器人重连次数达到上限100次，放弃重连");
                break;
            }
        }

        //监测IO板连接状态
        if(m_cTools.m_cIOA.getCommState() == false)
        {
            emit m_cTools.m_cIOA.sigDisconnected();
            qDebug()<<"m_cIOA.StopLoop();";
            qDebug()<<"尝试连接IO模块A。。。。"<<tryToConnectCount[1];
            isIOConnected = m_cTools.m_cIOA.ConnectToServer(g_str_IOAip, g_i_IOAport);
            if(tryToConnectCount[1] <= COUNT_LIMIT){
                ++tryToConnectCount[1];
            }else{
                log->error("IO模块重连次数达到上限100次，放弃重连");
                break;
            }
        }
        if(m_cTools.m_cIOB.getCommState() == false)
        {
            emit m_cTools.m_cIOB.sigDisconnected();
            qDebug()<<"m_cIOB.StopLoop();";
            qDebug()<<"尝试连接IO模块B。。。。"<<tryToConnectCount[2];
            isIOConnected = m_cTools.m_cIOB.ConnectToServer(g_str_IOBip, g_i_IOBport);
            if(tryToConnectCount[2] <= COUNT_LIMIT){
                ++tryToConnectCount[2];
            }else{
                log->error("IO模块重连次数达到上限100次，放弃重连");
                break;
            }
        }




       //启动遥控器com口 通过isOpen（）判断打开状态
       if(this->m_cManual.isOpen() == false){
           m_cManual.open("COM2");
           qDebug()<<"重新连接遥控器";
       }

        Sleep(1000);
    }
    qDebug()<<"cominterface： run() stopped";
}

void ComInterface::setLinkJointMoveAbs(uint index, double pos[],double vel[])
{
    double end_vel[100], distance[100];
    m_cRobot.setLinkGroupMove(index,eMC_MOV_ABS,pos,vel,end_vel,distance);
}

void ComInterface::setLinkJointMoveVel(uint index, double vel[])
{
    double pos[100], end_vel[100], distance[100];

    m_cRobot.setLinkGroupMove(index,eMC_MOV_VEL, pos, vel, end_vel, distance);

}


QVector<st_ReadAxis> ComInterface::getLinkJointStatus(uint index)
{
    return m_cRobot.getLinkJointStatus(index);
}

StatusofLink ComInterface::getLinkStatus(uint index)
{

    return m_cRobot.getLinkStatus(index);
}

void ComInterface::getManual(stManualCmd &m_Manual)
{
   this->m_cManual.getManualCmd(m_Manual);
}

void ComInterface::getManual(stManualOperator &m_Manual)
{
   this->m_cManual.getManualCmd(m_Manual);
}

void ComInterface::RobotReset()
{
    m_cRobot.RobotReset();
}

void ComInterface::RobotPower(bool enable)
{
    m_cRobot.RobotPower(enable);
}

void ComInterface::RobotHome()
{
    m_cRobot.RobotHome();
}

void ComInterface::RobotHalt()
{
    m_cRobot.RobotHalt();
}

void ComInterface::RobotStop()
{
    m_cRobot.RobotStop();
}


void ComInterface::LinkReset(uint index)
{
    m_cRobot.LinkReset(index);
}

void ComInterface::LinkPower(uint index, bool enable)
{
    m_cRobot.LinkPower(index, enable);
}

void ComInterface::LinkHome(uint index)
{
    m_cRobot.LinkHome(index);
}

void ComInterface::LinkHalt(uint index)
{
    m_cRobot.LinkHalt(index);
}

void ComInterface::LinkStop(uint index)
{
    m_cRobot.LinkStop(index);
}

void ComInterface::LinkMove(uint index, E_MotionMode mode, double pos[], double vel[], int freedom)
{
    m_cRobot.LinkMove(index, mode, pos, vel, freedom);
}

void ComInterface::JointPower(uint index, bool enable)
{
    m_cRobot.JointPower(index, enable);
}

void ComInterface::JointReset(uint index)
{
    m_cRobot.JointReset(index);
}

void ComInterface::JointHome(uint index, double pos)
{
    m_cRobot.JointHome(index, pos);
}

void ComInterface::JointHalt(uint index)
{
    m_cRobot.JointHalt(index);
}

void ComInterface::JointStop(uint index)
{
    m_cRobot.JointStop(index);
}

bool ComInterface::JointMove(uint index, eMC_Motion mode, double pos, double vel, double end_vel, double distance)
{
    return m_cRobot.JointMove(index, mode, pos, vel, end_vel, distance);
}

bool ComInterface::setJointGroupMove(eMC_Motion mode, const double pos[], const double vel[], double end_vel[], double distance[])
{
    return m_cRobot.setJointGroupMove(mode,pos,vel,end_vel,distance);
}


void ComInterface::setCrossLaser(bool swit)
{
    m_cTools.SetCrossLasser(swit);
}

void ComInterface::SetToolsAction(quint8 index, E_WeldAction action)
{
    m_cTools.SetToolsAction(index,action);
}

void ComInterface::SetMagentAction(quint8 index, E_MagentAction action)
{
    m_cTools.SetMagentAction(index,action);
}

void ComInterface::SetGunConnect(qint8 index)
{
    m_cTools.SetGunConnect(index);
}

void ComInterface::closeThread() {
    m_cTools.m_cIOA.closeThread();
    m_cTools.m_cIOB.closeThread();
     this->running = false;
     QThread::wait();
     qDebug()<<"cominterface： closeThread()";
}

bool ComInterface::getRobotConnectSta() {
    bool re;
    mutex_read.lock();
    re = this->m_cRobot.getCommState();
    mutex_read.unlock();
    return re;
}

bool ComInterface::getIOConnectSta(const uint index) {
    bool re;
    mutex_read.lock();
    if(index == 0){
        re = m_cTools.m_cIOA.getCommState();
    }else if(index == 1){
        re = m_cTools.m_cIOB.getCommState();
    }
    mutex_read.unlock();
    return re;
}

