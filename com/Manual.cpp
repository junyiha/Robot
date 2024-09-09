#include "Manual.h"


CManual::CManual()
{
    log = spdlog::get("logger");

    //com连接成功发送信号则打开loop，启动两个定时器；连接断开发送信号关闭loop。
    connect(this,&CSerialCom::sigConnected,this,&CManual::slotStartLoop);
    connect(this,&CSerialCom::sigDisconnected,this,&CManual::slotStopLoop);

    //定时器连接recvdata函数
    m_cManualRecvTimer=new QTimer(this);
    m_cManualRecvTimer->setInterval(90);
    connect(m_cManualRecvTimer,&QTimer::timeout,this,&CManual::RecvData);

}

void CManual::run()
{
    this->log->info("Manual thread start");
    while(this->isConnected ){
        if(this->isOpen()){
            SendDataRefactor();
            QThread::msleep(30);
            RecvDataRefactor();
        }else{
            this->log->error("CommState is false, please check io connection!");
        }
        QThread::msleep(20);
    }
    log->error("{}:{} CManual::run() 退出!!!", __FILE__, __LINE__);
}

int CManual::RecvData()
{
    uint8_t buffData[32];
    uint8_t readbuff[16];
    
    DWORD recvlen= 0;
    uint8_t check_xor;
    uint8_t check_plus;
    bool is_Valid = false;

    if(isOpen() == true)
    {
        // ReadFile获取到的数据段非理想协议帧的格式，因此根据数据传输协议的首位数据信息，进行筛选
        recvlen = read(buffData);

        if(recvlen>=16){
            for(int i=0;i<recvlen;i++){
                if(buffData[i] == 0x55){
                    std::copy(buffData+i,buffData+(i+16),readbuff);
                    is_Valid = true;
                    break;
                }
            }
         }



        if(recvlen >= 16 && is_Valid)
        {
            if(readbuff[0] == 0x55)//帧头校验
            {
                for(int i=0;i<14;i++)
                {
                    if(!i){
                        check_xor = readbuff[i];
                        check_plus = readbuff[i];
                    }
                    else{
                        check_xor ^= readbuff[i];
                        check_plus +=readbuff[i];
                    }
                }

                if(check_xor == readbuff[15] && check_plus == readbuff[14]) //校验
                {
                    //取数据
                    m_InputData.MoveVel   = static_cast<qint8>(readbuff[1]);

                    m_InputData.RotateVel = static_cast<qint8>(readbuff[2]);

                    m_InputData.BoardMoveVel = static_cast<qint8>(readbuff[3]);

                    m_InputData.RobotMove[0] = static_cast<qint8>(readbuff[4]);

                    m_InputData.RobotMove[1] = static_cast<qint8>(readbuff[5]);
                    m_InputData.RobotMove[2] = static_cast<qint8>(readbuff[6]);
                    m_InputData.RobotMove[3] = static_cast<qint8>(readbuff[7]);
                    m_InputData.RobotMove[4] = static_cast<qint8>(readbuff[8]);
                    m_InputData.RobotMove[5] = static_cast<qint8>(readbuff[9]);
                    m_InputData.MoveDirection = static_cast<quint8>(readbuff[10]);

                    //readbuff[11]未定义，跳过
                    m_InputData.EndMove = readbuff[12]&0b01000000; //0x40 readbuff[12]&0b00100000
                    m_InputData.Auto    = readbuff[12]&0b00100000; //0x20 0b10000000
                    m_InputData.Estop   = readbuff[13]&0b00000010; //0x02


                    Input2Cmd(); //转为指令
                    m_OutCmd.status = true;
                    mutex_cmd.lock();
                    std::memcpy(&_OutCmd,&m_OutCmd,sizeof(m_OutCmd));
                    mutex_cmd.unlock();

                    return recvlen;
                }
                else
                {
                    qDebug()<<"Check data failed, data is unused";
                }
            }else
            {
                qDebug()<<"Invalid data length, data is unused";
            }


        }else
        {
           qDebug()<<"Invalid data length, data is unused";
        }


    }else
    {
        emit sigDisconnected();
        mutex_cmd.lock();
        std::memset(&_OutCmd,0,sizeof(m_OutCmd));
        mutex_cmd.unlock();
        qDebug()<<"Port is not open";
    }


    //多次未收到指令，清空指令结构体
    static int failedcnt = 0;
    failedcnt ++;
    if(failedcnt > 3)
    {
        mutex_cmd.lock();
        std::memset(&_OutCmd,0,sizeof(m_OutCmd));
        mutex_cmd.unlock();

        failedcnt = 0;
    }
    return -1;
}

void CManual::getManualCmd(stManualCmd &cmd)
{
    mutex_cmd.lock();
    std::memcpy(&cmd, &_OutCmd, sizeof(stManualCmd));
    mutex_cmd.unlock();
}


void CManual::Input2Cmd()
{

    m_OutCmd.Estop      = m_InputData.Estop;
    m_OutCmd.Auto       = m_InputData.Auto;
    m_OutCmd.EndMove    = m_InputData.EndMove;

    if(m_OutCmd.Auto == false)
    {
        //车轮速度
        if(fabs(m_InputData.MoveVel/128.0) > Min_Thresh)
        {
            m_OutCmd.bMove = true;
            if(m_InputData.MoveVel>0)
            {
                m_OutCmd.MoveVel = (m_InputData.MoveVel/128.0-Min_Thresh) /(1-Min_Thresh);
            }else
            {
                m_OutCmd.MoveVel = (m_InputData.MoveVel/128.0+Min_Thresh) /(1-Min_Thresh);
            }

        }else
        {
            m_OutCmd.bMove = false;
            m_OutCmd.MoveVel = 0;

        }

        //舵轮方向 0~255对应 -PI/2~PI/2
        m_OutCmd.MoveDirection = (m_InputData.MoveDirection-128.0)/128*M_PI/2;

        //转向速度
        if(fabs(m_InputData.RotateVel/128.0) > Min_Thresh)
        {
            m_OutCmd.bRotate = true;
            if(m_InputData.MoveVel>0)
            {
                m_OutCmd.RotateVel = (m_InputData.MoveVel/128.0-Min_Thresh) /(1-Min_Thresh);
            }else
            {
                m_OutCmd.RotateVel = (m_InputData.MoveVel/128.0+Min_Thresh) /(1-Min_Thresh);
            }
        }
        else
        {
            m_OutCmd.bRotate = false;
            m_OutCmd.RotateVel = 0;
        }

        //运动到取/装板位
        if(fabs(m_InputData.BoardMoveVel/128.0) > Min_Thresh)
        {

            if(m_InputData.BoardMoveVel>0)
            {
                m_OutCmd.bPickBoard = false;
                m_OutCmd.bPushBoard = true;

            }else
            {
                m_OutCmd.bPickBoard = true;
                m_OutCmd.bPushBoard = false;
                m_OutCmd.MoveVel = (m_InputData.BoardMoveVel/128.0+Min_Thresh) /(1-Min_Thresh);
            }
            m_OutCmd.BoardMoveVel = (fabs(m_InputData.BoardMoveVel)/128.0-Min_Thresh) /(1-Min_Thresh);

        }else
        {
            m_OutCmd.bPickBoard = false;
            m_OutCmd.bPushBoard = false;
            m_OutCmd.BoardMoveVel = 0;

        }


        //单轴移动
        m_OutCmd.bRobotMove= false;
        for(int i=0;i<6;i++)
        {
            if(fabs(m_InputData.RobotMove[i]/128.0) > Min_Thresh)
            {
                m_OutCmd.bRobotMove= true;
                if(m_InputData.RobotMove[i]>0)
                {
                    m_OutCmd.RobotMove[i] = (m_InputData.RobotMove[i]/128.0-Min_Thresh) /(1-Min_Thresh);
                }else
                {
                   m_OutCmd.RobotMove[i] = (m_InputData.RobotMove[i]/128.0+Min_Thresh) /(1-Min_Thresh);
                }
            }
            else
            {
                m_OutCmd.RobotMove[i] = 0;
            }
        }
    }

}

void CManual::slotStartLoop()
{
    //m_cManualRecvTimer->start();
    this->start();
    this->isConnected = true;
}

void CManual::slotStopLoop()
{
    this->isConnected = false;
}

bool CManual::getConnectState() { // 判断遥控器连接情况
    // 正常连接情况: 1）串口可打开；2）可正确获取串口配置信息；3）已经触发startLoop()槽函数；

    bool flag = false;
    DCB p;
    bool ret = isOpen(); // 判断串口是否打开
    if(ret){
        if(GetCommState(hCom, &p)&isConnected){
            flag = true;
        }
    }
    return flag;
}