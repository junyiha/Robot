#ifndef CSERIALCOM_H
#define CSERIALCOM_H
#include <QObject>
#include <string>
#include <Windows.h>
#include <QDebug>


class CSerialCom:public QObject
{
    Q_OBJECT
public:
    CSerialCom();

    ~ CSerialCom();

    /**
      * @brief  开启串口
      * @param  char* portname：串口名称
      * @param  int baudrate：波特率
      * @param  char parity：校验方式
            0为无校验，1为奇校验，2为偶校验，3为标记校验
      * @param  char databit：数据位，4-8，通常为8位
      * @param  char stop_bit：停止位，1为1位停止位，2为2位停止位，3为1.5位停止位
      * @param  bool synchronizable：0为异步，1为同步
      * @return 正确返回true，错误返回false
      */
    bool open(const char* portname, int baudrate = 115200, char parity = 0,
              char databit = 8, char stopbit = 1, bool synchronizeflag = 1);
    /**
      * @brief  关闭串口
      * @param 无
      * @return 无
      */
    void close();

    /**
      * @brief   判断串口是否打开
      * @param 无
      * @return 打开返回true，关闭返回false
      */
    bool isOpen();

signals:
    void sigConnected();
    void sigDisconnected();

protected:
    /**
      * @brief   串口写入
      * @param const uint8_t* buffer：待写入的数据
      * @param DWORD length：待写入的数据长度
      * @return 发送的数据字节数，失败返回0
      */
    DWORD write(const uint8_t* buffer, DWORD length);

    /**
      * @brief   串口读取
      * @param uint8_t* buffer：待读取的数据
      * @param DWORD length：待读取的数据长度（默认32）
      * @return 读取的数据字节数，失败返回0
      */
    DWORD read(uint8_t* buffer, DWORD wCount = 32);
    HANDLE hCom;

private:
    bool synchronizeflag;  //synchronizable：0为异步，1为同步
};

#endif // CSERIALCOM_H
