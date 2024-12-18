/*****************************************************************
* 类名称： IOCom
* 功能描述： ·基于tcp接口建立iotcp接口，为统一接口的派生类
*          ·本版本中第一次使用，设计思想与RobotCom一致
* 参数说明： 参数说明
* 返回值：   返回值说明

* 数据协议--------------------------------------------------------
* 功能定义：握手机制,发送DO/AO输出数据（15byte），返回所有IO状态数据（32byte）
* 发送帧：
*          数据长度  DO数量 DI数量 AO数量 AI数量 硬件编号  DO数据     校验
*          BYTE0   BYTE1  BYTE2 BYTE3 BYTE4  BYTE5   BYTE7~13  BYTE14
*          0x0f    0x40   0x10  0x00   0x10  0x01/02  DO0~64    sum(0-13)
* 接收帧：
*          数据长度  DO数量 DI数量 AO数量 AI数量 硬件编号  DO数据     DI数据      AI数据        校验
*          BYTE0   BYTE1  BYTE2 BYTE3 BYTE5  BYTE6   BYTE7~13  BYTE14~15  BYTE16~47     BYTE48
*          0x0f    0x40  0x08  0x00   0x08  0x01/02  DO0~63    DI0~15     AI0~AI15     sum(0-47)
* 硬件功能--------------------------------------------------------
* 硬件清单：IO板A(编号01),B（编号02）, 工具10套，磁铁*4 磁铁推杆*4 十字激光*1 焊枪切换开关*10
* A板   工具1    工具2    工具3    工具4    工具5     磁铁吸合   推缸伸    推缸缩     十字激光  焊枪切换1~5
*       DO0~7  DO8~15   DO16~23  DO24~31 DO32~39  DO40,43  DO41,44  DO42,DO45   DO47   DO48~52
* a-1-5,b-6-10, A-激光
* B板   工具6    工具7    工具8    工具9    工具10    磁铁吸合   推缸伸    推缸缩              焊枪切换6~10
*       DO0~7  DO8~15   DO16~23  DO24~31 DO32~39  DO40,43  DO41,44  DO42,DO45          DO48~52
*
* 工具IO功能编号：  碰钉    打磨   定位气缸   打磨顶升  碰钉顶升   碰钉下降    打磨      碰钉
*                 0       1       2       3       4           5        6        7
******************************************************************/
#ifndef IOCOM_H
#define IOCOM_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QMutex>
#include <qvector.h>
#include <QAtomicInteger>

#include "../robot/DataStruct.h"
#include "TcpCom_IO.h"

#define SEND_LEN 0x0f
#define RECV_LEN 0x31

#define DO_LEN 8  // DO数组数据长度
#define DI_LEN 2  // DI数组数据长度
#define AI_NUM 16 // AI数量

class IOCom : public TcpCom_IO
{
    Q_OBJECT
public:
    IOCom();
    ~IOCom();

    /**
     * @brief SetIO 在上次DO命令基础上设置DO输出
     * @param index_port DO端子编号0~7   （共7个端子，从0开始）
     * @param value      端子输出状态（注意电气图纸是1-8编号）
     */
    void SetIO(qint8 index_port, unsigned char value);

    QVector<unsigned char> getDIState();
    QVector<unsigned char> getDOState();
    QVector<unsigned short> getAIState();
    void closeThread();
    bool is_Running = true;
    void run();

protected:
    //****************通信相关接口函数***************
    /**
     * @brief SendIO 发送数据
     * @return
     */
    DINT SendIO();

    /**
     * @brief RecvIO 接收数据
     * @return
     */
    DINT RecvIO();

    /**
     * @brief SendAndRecvIO 发送并接受
     * @return
     */
    DINT SendAndRecvIO();

    //****************子线程循环发送接口函数***************

protected slots:
    //*************子线程循环发送接口函数*************
    /**
     * @brief StartLoop 启动线程
     */

    void slotStartLoop();

    /**
     * @brief StopLoop 终止线程
     */
    void slotStopLoop();

private:
    unsigned char m_DOSet[DO_LEN];
    unsigned char m_DOState[DO_LEN];
    unsigned char m_DIState[DI_LEN];
    unsigned short m_AISate[AI_NUM];

    QTimer* m_cIOSendAndRecvTimer;

    QMutex mutex_send;
    QMutex mutex_recv;

    QAtomicInteger<INT64> m_SetIO;
    QAtomicInteger<INT64> m_readIO;
};

#endif // IOCOM_H
