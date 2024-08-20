#include "TcpCom_IO.h"

TcpCom_IO::TcpCom_IO()
{
    m_CommState = false;
    memset(m_SendData, 0, DATA_SIZE);
    memset(m_RecvData, 0, DATA_SIZE);
    m_SendSize = DATA_SIZE;
    m_RecvSize = DATA_SIZE;
    log = spdlog::get("logger");
}

TcpCom_IO::~TcpCom_IO()
{
    close();
}

int TcpCom_IO::ConnectToServer(const char *IpAdr, const int port)
{
    qDebug()<<"connect current threadID:"<<GetCurrentThreadId();
    unsigned long ul = 1;
    int i = 0;
    while (i++ < CONNECT_TIMES)
    {
        WSAStartup(MAKEWORD(2, 2), &wsaData);
        m_SockClient = socket(AF_INET, SOCK_STREAM, 0);
        //ioctlsocket(m_SockClient, FIONBIO, (unsigned long *)&ul);
        inet_pton(AF_INET, IpAdr, &(m_AdrServer.sin_addr));
        m_AdrServer.sin_family = AF_INET;
        m_AdrServer.sin_port = htons(port);

        qDebug()<<"try connect IO板";
        int re;
        re = ::connect(m_SockClient, (SOCKADDR*)&m_AdrServer, sizeof(SOCKADDR));
        qDebug()<<"connect over IO板";
        if (-1 == re)
        {
            int err = WSAGetLastError();
            printf("%d:%d\n", re, WSAGetLastError());
            if (err == WSAETIMEDOUT)
            {
                qDebug()<<"connect time out,trying again!\n ";
                Sleep(1000);
                continue;
            }
            else
            {
                qDebug()<<"connect err,give up\n ";
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
            emit sigConnected();
            return 0;
        }
    }
    return -1;

}

int TcpCom_IO::Sendbuffer()
{
    if (false == m_CommState)
    {
        qDebug()<<"no connection!";
        return -1;
    }
    //put data to buffer
    int sendnum = 0;
    ULONG datalen = 0;
    char sendbuff[DATA_SIZE];
    while ( datalen< m_SendSize)
    {
        memcpy( sendbuff, m_SendData + datalen,m_SendSize-datalen);
        //sendnum = send(m_SockClient, sendbuff, m_SendSize - datalen, 0);
        sendnum = send(m_SockClient, sendbuff, m_SendSize-datalen, 0);
        datalen = datalen + sendnum;
        if (-1 == sendnum)
        {
            int err = WSAGetLastError();
//            qDebug()<<err;
            if (err == EAGAIN || err == EWOULDBLOCK || err == EINPROGRESS)
            {
                continue;
            }
            else if (err == WSAETIMEDOUT)
            {
//                qDebug()<<"Send time out";
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
    //log->debug("IO_Sendbuff:{},{},{},{},{},",(quint8)sendbuff[6],(quint8)sendbuff[7],(quint8)sendbuff[8],(quint8)sendbuff[9],(quint8)sendbuff[10]);
    return 0;
}

DINT TcpCom_IO::Recvbuffer()
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
    while (datalen < m_RecvSize)
    {
        recvnum = recv(m_SockClient, recvbuff, m_RecvSize - datalen, 0);

        if (-1 == recvnum)
        {
            int err = WSAGetLastError();
//            qDebug()<<err;
            if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR)
            {
                continue;
            }
            else if(err == WSAETIMEDOUT)
            {
                cnt++;
//                qDebug()<<"recv time out";
                if (cnt > TIMEOUT_LIMIT)
                {
//                    qDebug()<<" IO板 give up recieve once";
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
        memcpy(m_RecvData + datalen, recvbuff, recvnum);
        datalen = datalen + recvnum;
    }
    //实际使用发现返回数据帧可能大于协议长度，因此在这里去除多余干扰数据
    char temp[5];
    recvnum = recv(m_SockClient, temp, 5, 0);
    return 0;
}


void TcpCom_IO::close()
{
    if(m_CommState == true)
    {
        qDebug()<<"close current threadID:"<<GetCurrentThreadId();
        qDebug()<<"socket is closed";
        closesocket(m_SockClient);
        WSACleanup();
        m_CommState = false;
        emit sigDisconnected();
    }
}
