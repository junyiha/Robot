#ifndef CMANUAL_H
#define CMANUAL_H

#include <QVector>
#include <QMutex>
#include "SerialCom.h"
#include <QTimer>
#include "ManualProtocol.h"

typedef struct DataofManual
{
    qint8 MoveVel;
    quint8 MoveDirection; //舵轮方向角
    qint8 RotateVel;
    qint8 BoardMoveVel;//前往装板/取板位置的速度
    qint8 RobotMove[6]; //机器人单轴/末端移动速度
    bool   EndMove;//true 末端移动，false，单轴移动
    bool   Auto;    //自动作业标志位
    bool   Estop;   //急停标志位
}stManualData;

typedef struct CommandfManual
{
    bool   Estop;   //急停标志位
    bool   Auto;    //自动作业标志位
    bool   EndMove;//true 末端移动，false，单轴移动

    double MoveVel; //车辆移动 速度百分比
    bool   bMove;

    double MoveDirection; //舵轮方向角度

    double RotateVel;      //车辆旋转速度百分比
    bool   bRotate;

    double BoardMoveVel;    //取装板移动速度百分比
    bool   bPickBoard;
    bool   bPushBoard;

    double RobotMove[6]; //机器人单轴/末端移动速度百分比
    bool   bRobotMove;
    bool   status;       // 命令是否有效标志位

}stManualCmd;

const double Min_Thresh = 0.3; //范围（0~0.9）推杆的最小推动阈值 大于行程Min_Thresh*100%是起作用

class CManual:public CSerialCom
{
public:
    CManual();

    /**
     * @brief RecvData 接收串口数据
     * @return [不确定]
     */
    int RecvData();

    /**
     * @brief 外部调用接口，获取遥控器指令
     * @param cmd
     */
    void getManualCmd(stManualCmd& cmd);

    bool getConnectState();
    bool isConnected = false;


protected:
    stManualData    m_InputData;
    stManualCmd     m_OutCmd;
    stManualCmd     _OutCmd;
    QTimer*         m_cManualRecvTimer;
    QMutex          mutex_cmd;
    /**
     * @brief 数据转换，将协议数据转换为命令m_InputData => m_OutCmd
     * @return 无
     */
    void Input2Cmd();

    //*************子线程循环发送接口函数*************
    /**
     * @brief StartLoop 子线程循环启动
     */
    void slotStartLoop();

    /**
     * @brief StopLoop 子线程循环停止
     */
    void slotStopLoop();

};

#endif // CMANUAL_H
