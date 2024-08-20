#include "SerialCom.h"


CSerialCom::CSerialCom()
{
    hCom = NULL;
    synchronizeflag = 1;
}
CSerialCom::~CSerialCom()
{
    close();
}
bool CSerialCom::open(const char* portname, int baudrate, char parity, char databit, char stopbit, bool synchronizeflag)
{
    //DWORD start = 0, stop = 0;
    //start = GetTickCount64();
    this->synchronizeflag = synchronizeflag;

    //同步方式
    if (this->synchronizeflag)
    {
        hCom = CreateFileA(portname, //串口名
                           GENERIC_READ | GENERIC_WRITE, //支持读写
                           0, //独占方式，串口不支持共享
                           NULL,//安全属性指针，默认值为NULL
                           OPEN_EXISTING, //打开现有的串口文件
                           0, //0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
                           NULL);//用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL
    }

    //异步方式
    else
    {
        hCom = CreateFileA(portname, //串口名
                           GENERIC_READ | GENERIC_WRITE, //支持读写
                           0, //独占方式，串口不支持共享
                           NULL,//安全属性指针，默认值为NULL
                           OPEN_EXISTING, //打开现有的串口文件
                           FILE_FLAG_OVERLAPPED, //0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
                           NULL);//用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL
    }

    if (hCom == (HANDLE)-1)
    {
        return false;
    }

    //配置缓冲区大小
    if (!SetupComm(hCom, 32, 32))
    {
        return false;
    }


    // 配置参数
    DCB p;
    if (!GetCommState(hCom, &p))//获得当前串口的配置信息
    {
        return false;
    }
    p.DCBlength = sizeof(p);
    p.BaudRate = baudrate; // 波特率
    p.ByteSize = databit; // 数据位

    switch (parity) //校验位
    {
    case 0:
        p.Parity = NOPARITY; //无校验
        break;
    case 1:
        p.Parity = ODDPARITY; //奇校验
        break;
    case 2:
        p.Parity = EVENPARITY; //偶校验
        break;
    case 3:
        p.Parity = MARKPARITY; //标记校验
        break;
    }

    switch (stopbit) //停止位
    {
    case 1:
        p.StopBits = ONESTOPBIT; //1位停止位
        break;
    case 2:
        p.StopBits = TWOSTOPBITS; //2位停止位
        break;
    case 3:
        p.StopBits = ONE5STOPBITS; //1.5位停止位
        break;
    }

    // 设置参数失败
    if (!SetCommState(hCom, &p))
    {
        return false;
    }

    //超时处理, 单位：毫秒
    // 总超时＝时间系数×读或写的字符数＋时间常量
    // 把间隔超时设为最大，把总超时设为0将导致ReadFile立即返回并完成操作
    // 51111-- sync
    // 10111-- non-sync
    if (this->synchronizeflag)
    {
        COMMTIMEOUTS timeouts_sync;
        timeouts_sync.ReadIntervalTimeout = 10; //读间隔超时
        timeouts_sync.ReadTotalTimeoutMultiplier = 5; //读时间系数
        timeouts_sync.ReadTotalTimeoutConstant = 5; //读时间常量
        timeouts_sync.WriteTotalTimeoutMultiplier = 1; // 写时间系数
        timeouts_sync.WriteTotalTimeoutConstant = 1; //写时间常量
        SetCommTimeouts(hCom, &timeouts_sync);
    }
    else
    {
        COMMTIMEOUTS timeouts_nsync;
        timeouts_nsync.ReadIntervalTimeout = 1; //读间隔超时
        timeouts_nsync.ReadTotalTimeoutMultiplier = 0; //读时间系数
        timeouts_nsync.ReadTotalTimeoutConstant = 1; //读时间常量
        timeouts_nsync.WriteTotalTimeoutMultiplier = 1; // 写时间系数
        timeouts_nsync.WriteTotalTimeoutConstant = 1; //写时间常量
        SetCommTimeouts(hCom, &timeouts_nsync);
    }


    //清空串口缓冲区
    PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);

    //stop = GetTickCount64();
    //printf("open: %d \n", (stop - start));

    emit sigConnected();
    return true;
}

void CSerialCom::close()
{
    if (hCom == NULL)
        return ;
    CloseHandle(hCom);
    hCom = NULL;
}

bool CSerialCom::isOpen()
{
    return hCom == NULL ? false : true;
}

DWORD CSerialCom::write(const uint8_t* buffer, DWORD length)
{
    //DWORD start = 0, stop = 0;
    //start = GetTickCount64();

    DWORD dwBytesWrite = length; //成功写入的数据字节数

    // 同步方式
    if (this->synchronizeflag)
    {
        BOOL bWriteStat = WriteFile(hCom, //串口句柄
                                    buffer, //数据首地址
                                    dwBytesWrite, //要发送的数据字节数
                                    &dwBytesWrite, //DWORD*，用来接收返回成功发送的数据字节数
                                    NULL); //NULL为同步发送，OVERLAPPED*为异步发送
        if (!bWriteStat)
        {
            return 0;
        }
    }

    //异步方式
    else
    {
        //		DWORD dwBytesWrite = length; //成功写入的数据字节数
        DWORD dwErrorFlags; //错误标志
        COMSTAT comStat; //通讯状态
        OVERLAPPED m_osWrite; //异步输入输出结构体

        //创建一个用于OVERLAPPED的事件处理，不会真正用到，但系统要求这么做
        memset(&m_osWrite, 0, sizeof(m_osWrite));
        m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, reinterpret_cast<LPCSTR>(L"WriteEvent"));

        ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误，获得设备当前状态
        BOOL bWriteStat = WriteFile(hCom, //串口句柄
                                    buffer, //数据首地址
                                    dwBytesWrite, //要发送的数据字节数
                                    &dwBytesWrite, //DWORD*，用来接收返回成功发送的数据字节数
                                    &m_osWrite); //NULL为同步发送，OVERLAPPED*为异步发送
        if (!bWriteStat)
        {
            if (GetLastError() == ERROR_IO_PENDING) //如果串口正在写入
            {
                WaitForSingleObject(m_osWrite.hEvent, 5); //等待写入事件5ms
            }
            else
            {
                ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误
                CloseHandle(m_osWrite.hEvent); //关闭并释放hEvent内存
                return 0;
            }
        }
    }
    //stop = GetTickCount64();
    //printf("write: %d \n", (stop - start));
    //return dwBytesWrite;
    return length;


}

DWORD CSerialCom::read(uint8_t* buffer, DWORD wCount)
{
    //	wCount = 32; //最大读取数据字节数
    if (this->synchronizeflag)
    {
        //同步方式
        BOOL bReadStat = ReadFile(
            hCom, //串口句柄
            buffer, //数据首地址
            wCount, //要读取的数据最大字节数
            &wCount, //DWORD*,用来接收返回成功读取的数据字节数
            NULL); //NULL为同步发送，OVERLAPPED*为异步发送
        if (!bReadStat)
            return 0;
        else
            return wCount;
    }
    else
    {
        //异步方式
        DWORD dwErrorFlags; //错误标志
        COMSTAT comStat; //通讯状态
        OVERLAPPED m_osRead; //异步输入输出结构体

        //创建一个用于OVERLAPPED的事件处理，不会真正用到，但系统要求这么做
        memset(&m_osRead, 0, sizeof(m_osRead));
        m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, reinterpret_cast<LPCSTR>(L"ReadEvent"));

        ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误，获得设备当前状态
        if (!comStat.cbInQue) return 0; //如果输入缓冲区字节数为0，则返回false
        //std::cout << comStat.cbInQue << std::endl;
        BOOL bReadStat = ReadFile(hCom, //串口句柄
                                  buffer, //数据首地址
                                  wCount, //要读取的数据最大字节数
                                  &wCount, //DWORD*,用来接收返回成功读取的数据字节数
                                  &m_osRead); //NULL为同步发送，OVERLAPPED*为异步发送
        if (!bReadStat)
        {
            if (GetLastError() == ERROR_IO_PENDING) //如果串口正在读取中
            {
                //GetOverlappedResult函数的最后一个参数设为TRUE
                //函数会一直等待，直到读操作完成或由于错误而返回
                GetOverlappedResult(hCom, &m_osRead, &wCount, TRUE);
            }
            else
            {
                ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误
                CloseHandle(m_osRead.hEvent); //关闭并释放hEvent的内存
                return 0;
            }
        }
        return wCount;
    }
}
