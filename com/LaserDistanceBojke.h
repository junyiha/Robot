//
// Created by csh_i on 2024/9/29.
//

#ifndef PDROBOT_MASTER_LASERDISTANCEBOJKE_H
#define PDROBOT_MASTER_LASERDISTANCEBOJKE_H
#include "SerialCom.h"
#include <spdlog/spdlog.h>
#include <QTimer>
#include <QMutex>
#include<iostream>



typedef struct LaserMeasureData {
    double m_Laserdistance[4] = {0};
    bool  m_bLaserdistance[4] = {false};
};


class LaserDistanceBojke: public CSerialCom{
public:
    LaserDistanceBojke();
    ~LaserDistanceBojke();
    LaserMeasureData m_LaserMeasureData;
    LaserMeasureData m_LaserMeasureData_copy;
    QMutex mutex;


    std::vector<uint8_t> laserID;



    void run();
    unsigned laserNums = 4;


    bool getConnectState();
    bool isConnected = false;
    std::shared_ptr<spdlog::logger> log;

    LaserMeasureData getLaserMeasureData();



protected:
    /**
     * @brief RecvData 接收串口数据
     * @return [不确定]
     */
    double RecvData();

    /**
     * @brief 发送串口数据
     * @return [不确定]
     */
    int SendData(int index);

    /**
     * @brief CRC校验计算
     * @return [0]
     */
     quint16 CTCCalculate(const uint8_t* data, uint16_t length);




    //*************子线程循环发送接口函数*************
    /**
     * @brief StartLoop 子线程循环启动
     */
    void slotStartLoop();

    /**
     * @brief StopLoop 子线程循环停止
     */
    void slotStopLoop();


    void sendAndRecvDataBatch();




};


#endif //PDROBOT_MASTER_LASERDISTANCEBOJKE_H
