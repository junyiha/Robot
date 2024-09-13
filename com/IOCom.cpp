#include "IOCom.h"

IOCom::IOCom()
{
    m_SendSize = SEND_LEN;
    m_RecvSize = RECV_LEN;
    memset(m_DOSet, 0, DO_LEN);
    memset(m_DOState, 0, DO_LEN);
    memset(m_DIState, 0, DI_LEN);
    memset(m_AISate, 0, AI_NUM*sizeof(unsigned short));

    //com???????????loop???????????????????loop?
    connect(this,&IOCom::sigConnected,this,&IOCom::slotStartLoop);
    connect(this,&IOCom::sigDisconnected,this,&IOCom::slotStopLoop);
//    //?????senddata??
//    m_cIOSendAndRecvTimer=new QTimer(this);
//    //500???200?? ,?????sleep
//    m_cIOSendAndRecvTimer->setInterval(100);
//    connect(m_cIOSendAndRecvTimer,&QTimer::timeout,this,&IOCom::SendAndRecvIO);



//    g_RecvIOLock.clear();
//    g_SendIOLock.clear();
}

IOCom::~IOCom()
{

}

void IOCom::slotStartLoop()
{
//    m_cIOSendAndRecvTimer->start();
    qDebug()<<" slotStartLoop start thread*************************************";
     this->start();
}

void IOCom::slotStopLoop()
{
//    m_cIOSendAndRecvTimer->stop();
}

DINT IOCom::SendIO()
{
    if(m_CommState == true)
    {
         mutex_send.lock();
        {
            m_SendData[0] = m_SendSize; //??
            m_SendData[1] = 64;			//DO??
            m_SendData[2] = 16;			//DI??
            m_SendData[3] = 0;			//AO??
            m_SendData[4] = 16;			//AI??
            m_SendData[5] = 1;			//????

            memcpy(&m_SendData[6], m_DOSet, DO_LEN); //DO??
                                                     //AO??????

            m_SendData[SEND_LEN-1] = 0;             //????
            for (int i = 0; i < SEND_LEN-1; i++)
            {
                m_SendData[SEND_LEN-1] = m_SendData[SEND_LEN-1] + m_SendData[i];
            }
        }
        mutex_send.unlock();

        //????
        int re = Sendbuffer();
        if (0 == re)
        {
            return 0;
        }
        else
        {
            qDebug()<<"failed to send";
            return -1;
        }
    }
    else
        return -10;
}

DINT IOCom::RecvIO()
{


    if(m_CommState == true)
    {
        DINT re = Recvbuffer();
        if (0 != re)
        {
            return -1;
        }

        mutex_recv.lock();
        {
            if (m_RecvData[0] != 0x31 || m_RecvData[1] != 0x40 || m_RecvData[2] != 0x10 || m_RecvData[4] != 0x10)
            {
                mutex_recv.unlock();
                return -1;
            }
            //log->debug("recv header check success");
            byte temp = 0;
            for (int i = 0; i < 0x30; i++)//????
            {
                temp = temp + static_cast<byte>(m_RecvData[i]);

            }
            if ((temp ^ static_cast<byte>(m_RecvData[0x30])) != 0x00)
            {
                mutex_recv.unlock();
                return -1;
            }
            //log->debug("recv sum check success");
            memcpy(m_DOState, &m_RecvData[6], DO_LEN);
            memcpy(m_DIState, &m_RecvData[6+DO_LEN], DI_LEN);
            memcpy(m_AISate, &m_RecvData[6+DO_LEN+DI_LEN], AI_NUM*sizeof(unsigned short));

            //?short??????????????????
            for(int i=0;i<AI_NUM;i++)
            {
                m_AISate[i] = ((m_AISate[i] << 8) | (m_AISate[i] >> 8));
//                qDebug()<<"origin m_AISate ; "<<m_AISate[i];
            }
        }
        mutex_recv.unlock();

        //log->debug("recv io action,m_AISate:{},{}",m_AISate[0],m_AISate[1]);

        return 0;
    }
    else
    {
        return -10;
    }
}

DINT IOCom::SendAndRecvIO()
{
    SendIO();
    QThread::msleep(400);
    RecvIO();
    return 0;
}

void IOCom::SetIO(qint8 index_port,byte value)
{
    if(index_port<DO_LEN)
    {
        mutex_send.lock();
        m_DOSet[index_port] = value;

        mutex_send.unlock();
    }else
    {
        qDebug()<<"error index_port";
    }

}

QVector<byte> IOCom::getDIState()
{
   mutex_recv.lock();
   QVector<byte> re(m_DIState,m_DIState+DI_LEN);
   mutex_recv.unlock();
   return  re;
}

QVector<byte> IOCom::getDOState()
{
    mutex_recv.lock();
    QVector<byte> re(m_DOState,m_DOState+DO_LEN);
    mutex_recv.unlock();
    return  re;
}

QVector<unsigned short> IOCom::getAIState()
{
    mutex_recv.lock();
    QVector<unsigned short> re(m_AISate,m_AISate+AI_NUM);
    mutex_recv.unlock();
    return  re;
}

void IOCom::run() {
    this->log->info("IOCom thread start");
    bool commState = this->getCommState();
    while(this->is_Running){
        if(commState){
            SendIO();
            QThread::msleep(200);
            RecvIO();
        }else{
            this->log->error("CommState is false, please check io connection!");
        }
    }
}

void IOCom::closeThread() {
    this->is_Running = false;
    QThread::wait();

}


