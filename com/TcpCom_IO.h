#ifndef TCPCOM2_H
#define TCPCOM2_H
/*****************************************************************
 * 函数名称： TcpCom2
 * 功能描述： ·基于以往项目的tcp的统一基类，定义了通用的成员和函数
 *          ·在本版本中修改为qobject派生类，增加少量信号和槽交互
 *          ·2版本中将接受逻辑改为超出预计接受长度则清空（清除干扰）
 * 参数说明： 参数说明
 * 返回值：   返回值说明
 ******************************************************************/

#pragma once
#include <QObject>
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <time.h>
#include <stdio.h>
#include <atomic>
#include <mutex>
#include <QTcpSocket>
#include <QDebug>
#include "../robot/DataStruct.h"
#include <spdlog/spdlog.h>
#include <QThread>
#pragma comment(lib, "ws2_32.lib")

#define CONNECT_TIMES 10
#define TIMEOUT_LIMIT 10
#define DATA_SIZE 5000

class TcpCom_IO : public QThread
{
    Q_OBJECT
public:
    TcpCom_IO();
    ~TcpCom_IO();

signals:
    void sigConnected();
    void sigDisconnected();

public slots:
    /**
     * @brief ConnectToServer
     * @param IpAdr
     * @param port
     * @return
     */
    int ConnectToServer(const char* IpAdr, const int port);

    /**
     * @brief close
     * @attention 本版本在主线程中直接子线程的此函数关闭连接，发现在点击关闭的时候同时在主线程和子线程调用了两次close
                  虽然不影响使用但是关闭两次不属于预期结果，因此在此加锁并判断状态字，只关闭一次连接，从表面上解决两次关闭问题
                  连接函数也是直接调用但是并无此问题，因此初步推定在某个类对象的析构中调用close了
     */
    void close();

public:
    /**
     * @brief Sendbuffer 发送缓冲
     * @return
     */
    int Sendbuffer();

    /**
     * @brief Recvbuffer 接收缓冲
     * @return
     */
    DINT Recvbuffer();

    bool getCommState() { return m_CommState; }

protected:
    bool m_CommState;
    ULONG m_SendSize;
    ULONG m_RecvSize;

    //
    char m_RecvData[DATA_SIZE];
    char m_SendData[DATA_SIZE];

    SOCKET m_SockClient;
    SOCKADDR_IN m_AdrServer;
    WSADATA wsaData;

    std::shared_ptr<spdlog::logger> log;
};

#endif // TCPCOM2_H
