#include "TcpCom.h"

TcpCom::TcpCom()
{
    m_CommState = false;
    memset(m_SendData, 0, DATA_SIZE);
    memset(m_RecvData, 0, DATA_SIZE);
    m_SendSize = DATA_SIZE;
    m_RecvSize = DATA_SIZE;

    log = spdlog::get("logger");
}

TcpCom::~TcpCom()
{
    if(m_CommState)
        close();
}

int TcpCom::ConnectToServer(const char *IpAdr, const int port)
{
    qDebug()<<"connect current threadID:"<<GetCurrentThreadId();
    unsigned long ul = 1;
    int i = 0;
    int finalRe = -1;

    while (i++ < CONNECT_TIMES)
    {
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        m_SockClient = socket(AF_INET, SOCK_STREAM, 0);
        inet_pton(AF_INET, IpAdr, &(m_AdrServer.sin_addr));
        m_AdrServer.sin_family = AF_INET;
        m_AdrServer.sin_port = htons(port);

        qDebug()<<"try connect Robot";
        int re;
        re = ::connect(m_SockClient, (SOCKADDR*)&m_AdrServer, sizeof(SOCKADDR));
        qDebug()<<"connect over Robot";
        if (-1 == re)
        {
            int err = WSAGetLastError();

            printf("%d:%d\n", re, WSAGetLastError());
            qDebug()<<"robot connection error is : "<<err;
            if (err == WSAETIMEDOUT)
            {
                qDebug()<<"connect time out,trying again!";
                Sleep(1000);
                continue;
            }
            else
            {
                qDebug()<<"connect err,give up";
                close();
                break;
            }
        }
        else
        {
            int SendTimeout = 10;   //1000ms
            int RecvTimeout = 10;   //1000ms
            setsockopt(m_SockClient, SOL_SOCKET, SO_RCVTIMEO, (char *)&RecvTimeout, sizeof(int));
            setsockopt(m_SockClient, SOL_SOCKET, SO_SNDTIMEO, (char *)&SendTimeout, sizeof(int));
            m_CommState = true;

            std::cout<<"m_CommState : "<<m_CommState<<std::endl;
            emit sigConnected();
            finalRe = 0;
            break;
        }
    }

    return finalRe;

}

int TcpCom::Sendbuffer()
{
    if (false == m_CommState)
    {
        qDebug()<<"no connection!\n";
        return -1;
    }
    //put data to buffer
    int sendnum = 0;
    ULONG datalen = 0;
    char sendbuff[DATA_SIZE];
    while ( datalen< m_SendSize)
    {
        memcpy( sendbuff, m_SendData + datalen,m_SendSize-datalen);
        sendnum = send(m_SockClient, sendbuff, m_SendSize - datalen, 0);
        datalen = datalen + sendnum;
        if (-1 == sendnum)
        {
            int err = WSAGetLastError();
            printf("%d\n", err);
            if (err == EAGAIN || err == EWOULDBLOCK || err == EINPROGRESS)
            {
                continue;
            }
            else if (err == WSAETIMEDOUT)
            {
                qDebug()<<"Send time out";
            }
            else
            {
                close();
                return -1;
            }
            continue;
        }
        else if (0 == sendnum)
        {
            close();
            return -1;
        }

    }
    return 0;
}

DINT TcpCom::Recvbuffer()
{
    if (false == m_CommState)
    {
        qDebug()<<"no connection!";
        return -1;
    }

    char recvbuff[DATA_SIZE];
    int recvnum = 0;
    ULONG datalen = 0;
    int cnt = 0;
    int num=0;
    while (datalen < m_RecvSize)
    {
        recvnum = recv(m_SockClient, recvbuff, 5*m_RecvSize - datalen, 0);

        if (-1 == recvnum)
        {
            int err = WSAGetLastError();          
            if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)
            {
                continue;
            }
            else if(err == WSAETIMEDOUT)
            {
                cnt++;                
                if (cnt > TIMEOUT_LIMIT)
                {
//                    qDebug()<<"机器人 give up recieve once";
                    return -2;
                }
            }
            else
            {
                close();
                m_CommState = false;
                return -1;
            }
            continue;
        }
        else if (0 == recvnum)
        {
            close();
            return -1;
        }
        if(recvnum > m_RecvSize)
        {
            num = m_RecvSize/5;
            recvnum = m_RecvSize%5;
            memcpy(m_RecvData + datalen, &recvbuff[5*m_RecvSize], recvnum);
        }else
        {
            memcpy(m_RecvData + datalen, recvbuff, recvnum);
            datalen = datalen + recvnum;
        }

    }
    return 0;
}

//注：本版本在主线程中直接子线程的此函数关闭连接，发现在点击关闭的时候同时在主线程和子线程调用了两次close
//虽然不影响使用但是关闭两次不属于预期结果，因此在此加锁并判断状态字，只关闭一次连接，从表面上解决两次关闭问题
//连接函数也是直接调用但是并无此问题，因此初步推定在某个类对象的析构中调用close了
void TcpCom::close()
{
    if(m_CommState == true)
    {
        log->warn("{}: close current threadID:", __LINE__, GetCurrentThreadId());
        log->warn("{}: socket is closed", __LINE__);
        closesocket(m_SockClient);
        WSACleanup();
        m_CommState = false;
        emit sigDisconnected();
    }
}
