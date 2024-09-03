#include "Manual.h"

int CManual::RecvDataRefactor()
{
    uint8_t buffData[32];
    uint8_t readbuff[16];
    
    DWORD recvlen= 0;
    uint8_t check_xor;
    uint8_t check_plus;
    bool is_Valid = false;

    if(!isOpen())
    {
        emit sigDisconnected();
        mutex_cmd.lock();
        std::memset(&_OutCmd,0,sizeof(m_OutCmd));
        mutex_cmd.unlock();
        qDebug()<<"Port is not open";
        return -1;
    }

    // ReadFile获取到的数据段非理想协议帧的格式，因此根据数据传输协议的首位数据信息，进行筛选
    recvlen = read(buffData);

    if(recvlen>=16)
    {
        for(int i=0;i<recvlen;i++)
        {
            if(buffData[i] == 0x55)
            {
                std::copy(buffData+i,buffData+(i+16),readbuff);
                is_Valid = true;
                break;
            }
        }
    }

    if(recvlen < 16 || !is_Valid)
    {
        qDebug()<<"Invalid data length, data is unused";
        return -1;
    }

    if(readbuff[0] != 0x55)//帧头校验
    {
        qDebug()<<"Invalid data length, data is unused";
        return -1;
    }

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

    if(check_xor != readbuff[15] || check_plus != readbuff[14]) //校验
    {
        qDebug()<<"Check data failed, data is unused";
        return -1;
    }

    ////////////////////////////////--新协议--//////////////////////////////////////////////
    m_manualOperator.TaskIndex = static_cast<quint8>(readbuff[7]);
    m_manualOperator.bEndMove = static_cast<int>(static_cast<quint8>(readbuff[8]) & 0b0000'0011) == 0 ? true : false;
    m_manualOperator.Ready = static_cast<int>(static_cast<quint8>(readbuff[8]) & 0b0000'1100);
    qint16 temp_val;
    temp_val = SpliceByte(static_cast<quint8>(readbuff[23]), static_cast<quint8>(readbuff[24]));
    m_manualOperator.VechVel =  temp_val / 100.0;
    m_manualOperator.bVechFlag = temp_val == 0 ? false : true;
    
    temp_val = SpliceByte(static_cast<quint8>(readbuff[25]), static_cast<quint8>(readbuff[26]));
    m_manualOperator.RotateVel = temp_val / 100.0;
    m_manualOperator.bRotateFlag = temp_val == 0 ? false : true;

    temp_val = SpliceByte(static_cast<quint8>(readbuff[9]), static_cast<quint8>(readbuff[10]));
    m_manualOperator.VechDirect = temp_val / 100.0;

    qint16 temp_val_x = SpliceByte(static_cast<quint8>(readbuff[11]), static_cast<quint8>(readbuff[12]));
    m_manualOperator.LinkMove.at(0) = temp_val_x / 100.0;
    qint16 temp_val_y = SpliceByte(static_cast<quint8>(readbuff[13]), static_cast<quint8>(readbuff[14]));
    m_manualOperator.LinkMove.at(1) = temp_val_y / 100.0;
    qint16 temp_val_z = SpliceByte(static_cast<quint8>(readbuff[15]), static_cast<quint8>(readbuff[16]));
    m_manualOperator.LinkMove.at(2) = temp_val_z / 100.0;

    qint16 temp_val_rx = SpliceByte(static_cast<quint8>(readbuff[17]), static_cast<quint8>(readbuff[18]));
    m_manualOperator.LinkMove.at(3) = temp_val_rx / 100.0;
    qint16 temp_val_ry = SpliceByte(static_cast<quint8>(readbuff[19]), static_cast<quint8>(readbuff[20]));
    m_manualOperator.LinkMove.at(4) = temp_val_ry / 100.0;
    qint16 temp_val_rz = SpliceByte(static_cast<quint8>(readbuff[21]), static_cast<quint8>(readbuff[22]));
    m_manualOperator.LinkMove.at(5) = temp_val_rz / 100.0;    

    if (temp_val_x || temp_val_y || temp_val_z || 
        temp_val_rx || temp_val_ry || temp_val_rz)
    {
        m_manualOperator.bLinkMoveFlag = true;
    }
    else 
    {
        m_manualOperator.bLinkMoveFlag = false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////

    //readbuff[11]未定义，跳过
    m_InputData.EndMove = readbuff[12]&0b01000000; //0x40 readbuff[12]&0b00100000
    m_InputData.Auto    = readbuff[12]&0b00100000; //0x20 0b10000000
    m_InputData.Estop   = readbuff[13]&0b00000010; //0x02


    Input2Cmd(); //转为指令
    m_OutCmd.status = true;
    mutex_cmd.lock();
    std::memcpy(&_OutCmd,&m_OutCmd,sizeof(m_OutCmd));
    mutex_cmd.unlock();

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

    return 0;
}

int CManual::SendDataRefactor()
{
    std::vector<uint8_t> buf(8, 0);

    buf[0] = buf.size();
    buf[1] = 0x04;
    buf[2] = 0x0F;
    buf[3] = 0x00;
    buf[4] = 0x0A;
    buf[5] = 0x01;
    buf[6] = 0x08;
    buf[7] = std::accumulate(buf.begin(), buf.end(), 0);

    return write(buf.data(), buf.size());
}

qint16 CManual::SpliceByte(qint8 high_byte, qint8 low_byte)
{
    return (static_cast<qint16>(high_byte) << 8) | low_byte;
}

void CManual::getManualCmd(stManualOperator& cmd)
{
    mutex_cmd.lock();
    

    std::memcpy(&cmd, &_manualOperator, sizeof(stManualOperator));

    mutex_cmd.unlock();
}