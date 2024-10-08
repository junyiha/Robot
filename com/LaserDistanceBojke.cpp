//
// Created by csh_i on 2024/9/29.
//

#include "LaserDistanceBojke.h"

LaserDistanceBojke::LaserDistanceBojke(){

    log = spdlog::get("logger");
    //com连接成功发送信号则打开loop，启动两个定时器；连接断开发送信号关闭loop。
    connect(this,&CSerialCom::sigConnected,this,&LaserDistanceBojke::slotStartLoop);
    connect(this,&CSerialCom::sigDisconnected,this,&LaserDistanceBojke::slotStopLoop);

}


bool LaserDistanceBojke::getConnectState() {
    bool flag = false;
    DCB p;
    bool ret = isOpen(); // 判断串口是否打开
    if(ret){
        if(GetCommState(hCom, &p)){
            flag = true;
        }
    }
    return flag;
}

LaserDistanceBojke::~LaserDistanceBojke() {
     if(hCom != NULL){
         delete hCom;
     }
}

void LaserDistanceBojke::run() {

    while(true) {
        if (isOpen()) {
            sendAndRecvDataBatch();
        }
        QThread::msleep(20);
    }
}

int LaserDistanceBojke::SendData(int index) {

    std::vector<uint8_t> buf(8, 0);

    buf[1] = 0x04;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x02;

    switch (index) {
        case 0:
            buf[0] = 0x01;
            buf[6] = 0x71;
            buf[7] = 0xCB;
            break;
        case 1:
            buf[0] = 0x02;
            buf[6] = 0x71;
            buf[7] = 0xF8;
            break;
        case 2:
            buf[0] = 0x03;
            buf[6] = 0x70;
            buf[7] = 0x29;
            break;
        case 3:
            buf[0] = 0x04;
            buf[6] = 0x71;
            buf[7] = 0x9E;
            break;
        default:
            return -1;
            break;
    }
    return write(buf.data(), buf.size());
}

void LaserDistanceBojke::sendAndRecvDataBatch() {
    memset(&m_LaserMeasureData, 0, sizeof(m_LaserMeasureData));
    for(int i = 0;i < laserNums;i++){
        SendData(i);
        QThread::msleep(10);
        double distance = RecvData();
        if(distance>0){
            m_LaserMeasureData.m_Laserdistance[i] = distance;
            m_LaserMeasureData.m_bLaserdistance[i] = true;
        }else{
            m_LaserMeasureData.m_Laserdistance[i] = 0.0;
            m_LaserMeasureData.m_bLaserdistance[i] = false;
        }
    }
    mutex.lock();
    std::memcpy(&m_LaserMeasureData_copy,&m_LaserMeasureData,sizeof(m_LaserMeasureData));
    mutex.unlock();
}

double LaserDistanceBojke::RecvData() {

    uint8_t buffData[100];
    uint8_t readbuff[100];

    DWORD recvlen= 0;
    uint32_t result = -1;

    if(!isOpen())
    {
        emit sigDisconnected();
        qDebug()<<"Port is not open";
        return -1;
    }

    recvlen = read(buffData);
    if(recvlen>=0x08){ // 接收数据长度大于8
        if((buffData[0]==0x01 | buffData[0]==0x02 |buffData[0]==0x03 |buffData[0]==0x04) && buffData[1]==0x04){ // 校验数据帧前两位
            result = (static_cast<uint32_t>(buffData[3])<<24) |
                     (static_cast<uint32_t>(buffData[4])<<16) |
                     (static_cast<uint32_t>(buffData[5])<<8) |
                      static_cast<uint32_t>(buffData[6]);

        }
    }
    if(result!=-1) {
        return result / 1000.0;
    }
    else{
        return -1.0;
    }
}

quint16 LaserDistanceBojke::CTCCalculate(const uint8_t *data, uint16_t length) {

    uint16_t crc = 0xFFFF; // 初始值
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= data[pos]; // 逐字节异或
        for (uint8_t i = 0; i < 8; i++) { // 逐位处理
            if (crc & 0x0001) {
                crc >>= 1; // 右移1位
                crc &= 0x7fff;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc; // 返回计算出的CRC
}

LaserMeasureData LaserDistanceBojke::getLaserMeasureData() {
    LaserMeasureData measureData;
    mutex.lock();
    std::memcpy(&measureData,&m_LaserMeasureData_copy,sizeof(m_LaserMeasureData));
    mutex.unlock();
    return measureData;
}

void LaserDistanceBojke::slotStartLoop() {
//    std::cout << "LaserDistanceBojke start loop" << std::endl;
    this->start();
//    this->log->info("LaserDistanceBojke start loop");
}

void LaserDistanceBojke::slotStopLoop() {
    this->quit();
}
