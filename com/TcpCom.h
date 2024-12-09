/*****************************************************************
 * 函数名称： TcpCom
 * 功能描述： ·基于以往项目的tcp的统一基类，定义了通用的成员和函数
 *          ·在本版本中修改为qobject派生类，增加少量信号和槽交互
 * 参数说明： 参数说明
 * 返回值：   返回值说明
 ******************************************************************/
#ifndef TCPCOM_H
#define TCPCOM_H

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
#include <iostream>
#include <spdlog/spdlog.h>

#include "robot/DataStruct.h"
#pragma comment(lib, "ws2_32.lib")

#define CONNECT_TIMES 10
#define TIMEOUT_LIMIT 10

#ifndef TCPCOM_DATA_SIZE
#define TCPCOM_DATA_SIZE 20000
#endif  // TCPCOM_DATA_SIZE

class TcpCom : public QObject
{
    Q_OBJECT
public:
    TcpCom();
    ~TcpCom();

public:
    /**
     * @brief ConnectToServer
     * @param IpAdr
     * @param port
     * @return
     */
    int ConnectToServer(const char* IpAdr, const std::size_t port);

    /**
     * @brief close
     */
    void close();

public:
    /**
     * @brief Sendbuffer
     * @return
     */
    int Sendbuffer();

    /**
     * @brief Recvbuffer
     * @return
     */
    DINT Recvbuffer();

    /**
     * @brief 获取通讯状态
     * @return
     */
    bool getCommState() { return m_CommState; }

protected:
    bool m_CommState;
    ULONG m_SendSize;
    ULONG m_RecvSize;

    char m_RecvData[TCPCOM_DATA_SIZE];
    char m_SendData[TCPCOM_DATA_SIZE];

    SOCKET m_SockClient;
    SOCKADDR_IN m_AdrServer;
    WSADATA wsaData;

    std::shared_ptr<spdlog::logger> log;

signals:
    void sigConnected();
    void sigDisconnected();
};

#endif // TCPCOM_H
