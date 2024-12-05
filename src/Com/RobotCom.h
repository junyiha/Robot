/*****************************************************************
 * 函数名称： RobotCom
 * 功能描述： ·基于tcp接口建立robottcp接口，在以往项目中为统一接口的派生类
 *          ·在本版本中提升为机器人的对象，不仅包括tcp接口还包括控制命令的调用
 *          ·采取槽函数的形式方便使用信号调用，在多线程下可直接movetothread移入线程
 * 参数说明： 参数说明
 * 返回值：   返回值说明
 ******************************************************************/
#ifndef ROBOTCOM_H
#define ROBOTCOM_H

#include <QObject>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QAtomicInt>
#include <QAtomicPointer>
#include <QMutex>

#include "TcpCom.h"

#define endPosAdjMaxCount 100

class RobotCom : public TcpCom
{
    Q_OBJECT
public:
    RobotCom();
    ~RobotCom();

    // 接收状态信息
    DINT m_RobotHeartbeat;
    stRobotStatus m_RobotStaus;
    stLinkStatus m_LinkStatus[MAX_FREEDOM_LINK];
    st_ReadAxis m_AxisStatus[MAX_FREEDOM_ROBOT];
    // 控制指令信息
    DINT m_UpperHeartbeat;
    stRobotCommand m_RobotCmmd;
    stLinkCommand m_LinkCommd[MAX_FREEDOM_LINK];
    st_SetAxis m_AxisCommd[MAX_FREEDOM_ROBOT];

private:
    ULONG m_LinkNum = 0;
    ULONG m_JointNum = 0;
    SYSTEMTIME m_SysTime;
    ULONG m_Hour;
    ULONG m_endhour;

    QTimer *m_crobotSendTimer;
    QTimer *m_crobotRecvTimer;
    QTimer *m_endPosSendTimer;

    QMutex mutex_send;
    QMutex mutex_Read;

signals:
    void sigRobotDataUpdate();

public:
    /**
     * @brief setJointMotionRT 实时运动控制函数 高频发送运动控制命令接口 单轴
     * @param joint_index 轴索引
     * @param mode 运动模式
     * @param pos 位置
     * @param vel 速度
     * @param end_vel 结束速度
     * @param distance 方向
     * @return [不确定]
     */
    // bool setJointMotionRT(uint joint_index, eMC_Motion mode, double pos, double vel, double end_vel, double distance);

public slots:
    //****************通信相关接口函数***************
    /**
     * @brief RecvData 接收数据
     * @return [不确定]
     */
    DINT RecvData();

    /**
     * @brief SendData 发送数据
     * @return [不确定]
     */
    DINT SendData();

    /**
     * @brief GetTime 获取时间[不确定]
     * @return [不确定]
     */
    DINT GetTime();

    //*************子线程循环发送接口函数*************
    /**
     * @brief StartLoop 子线程循环启动
     */
    void slotStartLoop();

    /**
     * @brief StopLoop 子线程循环停止
     */
    void slotStopLoop();

public:
    //*************机器人控制相关接口函数*************
    /**
     * @brief EndVelMove 末端速度函数 控制末端速度
     * @param i
     * @param vel 速度
     * @param direction 方向
     * @param underTool 是否在工具系下
     * @attention 函数实现需要下位机编写相关程序
     */
    void EndVelMove(uint i, double vel = 1, bool direction = true, bool underTool = false);

    /**
     * @brief EndPosMove 末端位置函数 控制末端位置
     * @param dp
     * @param underTool 是否在工具系下
     * @attention 函数实现需下位机编写相关程序
     */
    void EndPosMove(double *dp, bool underTool = false);

    /**
     * @brief RobotReset 机器人重置函数 控制整体运动
     */
    void RobotReset();

    /**
     * @brief RobotPower 机器人使能函数 控制整体运动
     * @param enable 是否使能
     * @attention 原定参数无误返回true ，但目前是void
     */
    void RobotPower(bool enable);

    /**
     * @brief RobotHome 机器人返回零位 控制整体运动
     * @attention 原定参数无误返回true
     */
    void RobotHome();

    /**
     * @brief RobotHalt 机器人暂停函数 控制整体运动
     * @attention 原定参数无误返回true
     */
    void RobotHalt();

    /**
     * @brief RobotStop 机器人急停函数 控制整体运动
     * @attention 原定参数无误返回true
     */
    void RobotStop();

    /**
     * @brief RobotMove 机器人运动函数 控制整体运动
     * @param mode 电机运动模式
     * @param pos 位姿
     * @param vel 速度
     * @param freedom 自由度
     */
    // void RobotMove(E_MotionMode mode, double pos[], double vel[], uint freedom);

    //************臂/车控制相关接口函数*************

    /**
     * @brief LinkReset link重置函数
     * @param index 轴索引序号
     * @attention 原定参数无误返回true
     */
    void LinkReset(uint index);

    /**
     * @brief LinkPower link使能函数
     * @param index 轴索引序号
     * @param enble 使能
     * @attention 原定参数无误返回true
     */
    void LinkPower(uint index, bool enble);

    /**
     * @brief LinkHome link回归零位
     * @param index 轴索引序号
     * @attention 原定参数无误返回true
     */
    void LinkHome(uint index);

    /**
     * @brief LinkPower link暂停函数
     * @param index 轴索引序号
     * @attention 原定参数无误返回true
     */
    void LinkHalt(uint index);

    /**
     * @brief LinkPower link急停函数
     * @param index 轴索引序号
     * @attention 原定参数无误返回true
     */
    void LinkStop(uint index);

    /**
     * @brief LinkMove link工具运动函数
     * @param index 轴索引
     * @param mode 模式
     * @param pos 位姿
     * @param vel 速度
     * @param freedom 自由度
     */
    void LinkMove(uint index, E_MotionMode mode, double pos[], double vel[], int freedom);

    //************单轴控制相关接口函数*************

    /**
     * @brief JointPower 单轴使能函数
     * @param index 轴索引
     * @param enble 使能
     * @attention 原定参数无误返回true
     */
    void JointPower(uint index, bool enble);

    /**
     * @brief JointReset 单轴重置函数
     * @param index 轴索引
     * @attention 原定参数无误返回true
     */
    void JointReset(uint index);

    /**
     * @brief JointHome 单轴回零函数
     * @param index 轴索引
     * @param pos 位姿
     * @attention 原定参数无误返回true
     */
    void JointHome(uint index, double pos);

    /**
     * @brief JointHalt 单轴暂停函数
     * @param index 轴索引
     * @attention 原定参数无误返回true
     */
    void JointHalt(uint index);

    /**
     * @brief JointStop 单轴急停函数
     * @param index 轴索引
     * @attention 原定参数无误返回true
     */
    void JointStop(uint index);

    /**
     * @brief JointMove 单轴运动函数
     * @param index 轴索引
     * @param mode 运动模式 eMC_MOV_ABS,   eMC_MOV_CON_ABS,    eMC_MOV_RELATIVE,   eMC_MOV_VEL
     * @param pos 选填，目标位置，eMC_MOV_ABS，eMC_MOV_CON_ABS 模式有效
     * @param vel 选填，运动速度, eMC_MOV_ABS，eMC_MOV_VEL 模式有效
     * @param end_vel 选填，末尾点速度,eMC_MOV_CON_ABS模式有效模式有效
     * @param distance 选填，相对运动距离，eMC_MOV_RELATIVE模式有效
     * @return 参数无误返回true
     */
    bool JointMove(uint index, eMC_Motion mode, double pos = 0, double vel = 0, double end_vel = 0, double distance = 0);

    /**
     * @brief 轴组实时运动控制函数 高频发送运动控制命令接口 多轴
     * @param joint_index 轴索引
     * @param mode 运动模式
     * @param pos 位置
     * @param vel 速度
     * @param end_vel 结束速度
     * @param distance 方向
     * @return [不确定]
     */
    bool setJointGroupMove(eMC_Motion mode, const double pos[], const double vel[], double end_vel[], double distance[]);

    /**
     * @brief LIN轴组实时运动控制函数 高频发送运动控制命令接口 多轴
     * @param index LINK索引
     * @param mode 运动模式
     * @param pos 位置:与link自由度一致
     * @param vel 速度:与link自由度一致
     * @param end_vel 结束速度:与link自由度一致
     * @param distance 方向:与link自由度一致
     * @return [不确定]
     */
    bool setLinkGroupMove(uint index, eMC_Motion mode, const double pos[], double vel[], double end_vel[], double distance[]);

    //*************读机器人状态接口函数*************
    /**
     * @brief 读Link内轴组数据
     * @param index link索引
     * @return 轴组状态向量
     */
    QVector<st_ReadAxis> getLinkJointStatus(uint index);

    /**
     * @brief 读Link状态
     * @param index link索引
     * @return link状态向量
     */
    StatusofLink getLinkStatus(uint index);

    /**
     * @brief 读所有轴组数据
     * @return 轴组状态向量
     */
    QVector<st_ReadAxis> getJointGroupStatus();
};

#endif // ROBOTCOM_H
