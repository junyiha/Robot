#include "Manual.h"

int CManual::RecvDataRefactor()
{
    uint8_t buffData[100];
    uint8_t readbuff[100];

    DWORD recvlen = 0;
    uint8_t check_plus{0};
    bool is_Valid = false;

    if (!isOpen())
    {
        emit sigDisconnected();
        mutex_cmd.lock();
        std::memset(&_manualOperator, 0, sizeof(m_manualOperator));
        mutex_cmd.unlock();
        qDebug() << "Port is not open";
        return -1;
    }

    // ReadFile获取到的数据段非理想协议帧的格式，因此根据数据传输协议的首位数据信息，进行筛选
    recvlen = read(buffData);

    if (recvlen >= 0x1D)
    {
        for (int i = 0; i <= recvlen - 0x1D; i++)
        {
            if (buffData[i] == 0x1D)
            {
                std::copy(buffData + i, buffData + (i + 0x1D), readbuff);
                is_Valid = true;
                break;
            }
        }
    }

    static int failedcnt = 0;

    if (failedcnt > 3)
    {
        mutex_cmd.lock();
        std::memset(&_manualOperator, 0, sizeof(_manualOperator));
        mutex_cmd.unlock();

        failedcnt = 0;
    }

    if (recvlen < 0x1D || !is_Valid)
    {
        log->warn("Invalid data length, data is unused");
        failedcnt++;
        return -1;
    }

    for (int i = 0; i < readbuff[0]; i++)
    {
        check_plus += readbuff[i];
    }

    if (check_plus == readbuff[0x1C]) // 校验
    {
        log->warn("Check data failed, data is unused");
        failedcnt++;
        return -1;
    }

    ////////////////////////////////--新协议--//////////////////////////////////////////////
    m_manualOperator.TaskIndex = static_cast<quint8>(readbuff[6]);
    m_manualOperator.bEndMove = static_cast<int>(static_cast<quint8>(readbuff[7]) & 0b0000'0011) == 2 ? true : false; // 修改1
    m_manualOperator.Ready = static_cast<int>(static_cast<quint8>(readbuff[7]) & 0b0000'1100) >> 2;                   // 修改2
    std::pair<double, bool> temp_val;
    temp_val = translateToVelocity(static_cast<quint8>(readbuff[22]), static_cast<quint8>(readbuff[23]));
    m_manualOperator.VechVel = temp_val.first;
    m_manualOperator.bVechFlag = temp_val.second;

    temp_val = translateToVelocity(static_cast<quint8>(readbuff[24]), static_cast<quint8>(readbuff[25]));
    m_manualOperator.RotateVel = temp_val.first;
    m_manualOperator.bRotateFlag = temp_val.second;

    m_manualOperator.VechDirect = (static_cast<quint8>(readbuff[8]) * 256 + static_cast<quint8>(readbuff[9]) - 2048) / 2048.0 * 100;

    auto temp_val_x = translateToVelocity(static_cast<quint8>(readbuff[10]), static_cast<quint8>(readbuff[11]));
    m_manualOperator.LinkMove.at(0) = temp_val_x.first;
    auto temp_val_y = translateToVelocity(static_cast<quint8>(readbuff[12]), static_cast<quint8>(readbuff[13]));
    m_manualOperator.LinkMove.at(1) = temp_val_y.first;
    auto temp_val_z = translateToVelocity(static_cast<quint8>(readbuff[14]), static_cast<quint8>(readbuff[15]));
    m_manualOperator.LinkMove.at(2) = temp_val_z.first;

    auto temp_val_rx =
        translateToVelocity(static_cast<quint8>(readbuff[16]), static_cast<quint8>(readbuff[17]));
    m_manualOperator.LinkMove.at(3) = temp_val_rx.first;
    auto temp_val_ry =
        translateToVelocity(static_cast<quint8>(readbuff[18]), static_cast<quint8>(readbuff[19]));
    m_manualOperator.LinkMove.at(4) = temp_val_ry.first;
    auto temp_val_rz =
        translateToVelocity(static_cast<quint8>(readbuff[20]), static_cast<quint8>(readbuff[21]));
    m_manualOperator.LinkMove.at(5) = temp_val_rz.first;

    if (temp_val_x.second || temp_val_y.second || temp_val_z.second ||
        temp_val_rx.second || temp_val_ry.second || temp_val_rz.second)
    {
        m_manualOperator.bLinkMoveFlag = true;
    }
    else
    {
        m_manualOperator.bLinkMoveFlag = false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////

    // readbuff[11]未定义，跳过
    // m_InputData.EndMove = readbuff[12]&0b01000000; //0x40 readbuff[12]&0b00100000
    // m_InputData.Auto    = readbuff[12]&0b00100000; //0x20 0b10000000
    // m_InputData.Estop   = readbuff[13]&0b00000010; //0x02

    // Input2Cmd(); //转为指令
    // m_OutCmd.status = true;
    mutex_cmd.lock();
    std::memcpy(&_manualOperator, &m_manualOperator, sizeof(m_manualOperator)); // 修改
    mutex_cmd.unlock();

    // 多次未收到指令，清空指令结构体

    return 0;
}

int CManual::SendDataRefactor()
{
    std::vector<uint8_t> buf(7, 0);

    buf[0] = 0x07;
    buf[1] = 0x00;
    buf[2] = 0x0C;
    buf[3] = 0x00;
    buf[4] = 0x0A;
    buf[5] = 0x01;
    buf[6] = 0X1E;

    return write(buf.data(), buf.size());
}

std::pair<double, bool> CManual::translateToVelocity(quint8 high_byte, quint8 low_byte)
{
    qint16 tmp = high_byte * 256 + low_byte - 2048;
    if (tmp > 200)
    {
        return std::make_pair((tmp - 200) / 1500.0, true);
    }
    else if (tmp < -200)
    {
        return std::make_pair((tmp + 200) / 1500.0, true);
    }
    else
    {
        return std::make_pair(0.0, false);
    }
}

void CManual::getManualCmd(stManualOperator &cmd)
{
    mutex_cmd.lock();

    cmd = _manualOperator;
    // std::memcpy(&cmd, &_manualOperator, sizeof(stManualOperator));

    mutex_cmd.unlock();
}