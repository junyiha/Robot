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

void CManual::ReadDataTest()
{
    std::vector<qint8> data;

    qint8 arrSize = static_cast<qint8>(data[0]);
    qint8 DOSsize = static_cast<qint8>(data[1]);
    qint8 DISsize = static_cast<qint8>(data[2]);
    qint8 AOSsize = static_cast<qint8>(data[3]);
    qint8 AISsize = static_cast<qint8>(data[4]);
    qint8 HardwareCode = static_cast<qint8>(data[5]);

}