#ifndef ROBOT_H
#define ROBOT_H
/* *****************************************************************************
 * CRobot:包含link0(机械臂)link1 link2(底盘)
 * LINK0: //1前后平台//2左右平台//3底部升降//4腰旋转//5腰俯仰
 *        //6推杠俯仰//7炮筒伸缩 8炮筒自转//9末端俯仰//10末端升降
 * 其中[5腰俯仰]因机械因素需单独处理，这部分在com部分进行
 * LIN2:底盘包括两个行走轮和两个舵轮
 *
 *
 *
 ******************************************************************************/

#include <QThread>
#include <QTime>
#include <qmath.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ComInterface.h"
#include "RobotKinectModel.h"
#include "Measure.h"

class CRobot : public QThread
{
    Q_OBJECT
public:
    explicit CRobot(ComInterface *comm, QObject *parent = nullptr);
    ~CRobot();

    /**
     * @brief 写入机器人指令
     * @param cmd 指令状态
     */
    void setLinkCom(stLinkCommand cmd);

    /**
     * @brief 获取机器人LINK状态
     * @param sta 机器人状态数据
     */
    stLinkStatus getLinkSta();

    /**
     * @brief 获取机器人各个关节状态
     * @return 轴组状态数据
     */
    QVector<st_ReadAxis> getJointGroupSta();

    void closeThread();

protected:
    /**
     * @brief 重载运行函数
     */
    void run() override;

    /**
     * @brief 机器人状态计算
     */
    void UpdateStatus();

    /**
     * @brief 机器人状态机
     */
    void StateMachine();

    /**
     * @brief 机器人子状态机_联动
     */
    void StateMachineMove();

    int m_Index;
    int m_Freedom; // 机器人自由度，不包括工具、底盘
    int m_ToolFreedom;
    CRobotKinectModel *m_LinkModel;

    ComInterface *m_Comm;  // 通讯模块索引
    bool m_running = true; // 运行状态标志/控制位

    stLinkCommand m_LinkCmd; // 任务部分下发的指令
    stLinkCommand _LinkCmd;  // 任务部分下发的指令,数据交互用
    st_AxisGroupSet m_AxisCmd;
    st_AxisGroupSet _AxisCmd;

    stLinkStatus m_LinkSta; // 轴状态
    stLinkStatus _LinkSta;  // 轴状态,,数据交互用
    double m_ActPose[6];    // 基坐标系下的位姿,mm、rad
    double m_ActJoints[20]; // 关节位置
    QVector<st_ReadAxis> m_JointGroupStatus;
    QVector<st_ReadAxis> _JointGroupStatus;

    Eigen::Matrix4d _EndPose; // 末端位姿矩阵

    std::shared_ptr<spdlog::logger> log;

private:
    double m_JointVelLimt[20]; // 关节限速，单位弧度/s
    double m_JointPosLimt[20]; // 关节正限位，单位弧度
    double m_JointNegLimt[20]; // 关节负限位，单位弧度
    QMutex mutex_cmd;          // 写指令保护锁
    QMutex mutex_sta;          // 读状态保护锁
    QMutex mutex_stajoint;     // 读状态保护锁
    QMutex mutex_posDev;       // 读写指令保护锁

    /**
     * @brief 沿直线移动到目标位置--基坐标
     * @param car_pos 笛卡尔坐标_基坐标_目标位置
     * @param vel_max 单维度最大速度
     * @attention 内部未做数组越界保护，确保输入变量为6维数组
     */
    void MoveLineAbsBase(Eigen::Matrix4d tarpose, double vel_max[]);

    /**
     * @brief 沿直线移动到目标位置--基坐标
     * @param deltapose[6] 工具坐标末端位姿偏差
     * @param vel_max[6]   单维度最大速度
     * @attention 内部未做数组越界保护，确保输入变量为6维数组
     */
    void MoveLineAbsBase(Eigen::VectorXd car_pos, Eigen::VectorXd vel_max);

    /**
     * @brief 沿给定速度方向运动--工具坐标系
     * @param car_vel[6] 笛卡尔坐标_基坐标_目标速度
     */
    void MoveLineVelTool(Eigen::VectorXd car_vel);

    /**
     * @brief 沿给定速度方向运动--关节空间
     * @param joint_vel 单轴目标速度 机器人关节+工具电机
     */
    void MoveJointVel(Eigen::VectorXd joint_vel);

    /**
     * @brief 机器人移动到目标位置--关节空间，各关节同时到达
     * @param joint_pos 关节目标位置size<20
     * @param joint_vel 关节目标速度size<20
     */
    void MoveJointAbs(Eigen::VectorXd joint_pos, Eigen::VectorXd joint_vel);

public:
    /** ***********外部接口函数，供外部调用修改m_RobotCmd,m_AxisCmd*************/
    /**
     * @brief 停止所有轴运动
     * @param null
     */
    void setRobotStop();

    /**
     * @brief 停止运动外部接口，通过调用存入外部指令
     * @param null
     */
    void setRobotHalt();

    /**
     * @brief 使能外部接口，通过调用存入外部指令
     * @param enbale 机器人上使能标志位
     * @param index Link索引
     */
    void setRobotEnable(bool enable);

    /**
     * @brief 清错外部接口，通过调用存入外部指令
     * @param null
     */
    void setRobotReset();

    /**
     * @brief 使能外部接口，通过调用存入外部指令
     * @param enbale 机器人上使能标志位
     * @param index Link索引
     */
    void setLinkEnable(bool enable);

    /**
     * @brief 清错外部接口，通过调用存入外部指令
     * @param index Link索引
     * @param null
     */
    void setLinkReset();

    /**
     * @brief 停止运动外部接口，通过调用存入外部指令
     * @param null
     */
    void setLinkHalt();

    /**
     * @brief 急停外部接口，通过调用存入外部指令
     * @param null
     */
    void setLinkStop();

    /**
     * @brief 单轴速度运动外部控制接口 move_vel
     * @param index 轴索引
     * @param vel   运动速度
     */
    void setJointMoveVel(uint index, double vel);

    /**
     * @brief 单轴绝对运动外部控制接口 move_abs
     * @param index 轴索引
     * @param pos   目标位置
     * @param vel   运动速度
     */
    void setJointMoveAbs(uint index, double pos, double vel);

    /**
     * @brief 多轴速度运动外部控制接口 move_vel
     * @param vel   运动速度
     */
    void setJointGroupMoveVel(double vel[]);

    /**
     * @brief 多轴绝对运动外部控制接口 move_abs
     * @param pos   目标位置
     * @param vel   运动速度
     */
    void setJointGroupMoveAbs(const double pos[], const double vel[]);

    /**
     * @brief 末端模式速度运动控制接口 move_vel
     * @param vel[]   运动速度
     */
    void setLinkMoveVel(double vel[]);

    /**
     * @brief 末端模式绝对运动控制接口 move_abs
     * @param pos[]   目标位置
     * @param vel[]   运动速度 mm,rad
     */
    void setLinkMoveAbs(const double pos[], const double vel[]);

    /**
     * @brief 根据目标位置在工具坐标系下的偏差矩阵，计算目标在基坐标系下的位置
     * @param tar_rt   目标位置偏差矩阵（工具坐标系）
     * @return  目标位置（基坐标系）
     */
    QVector<double> getTargetPose(const Eigen::Matrix4d tar_rt);

    /**
     * @brief 是否达到目标关节位置
     * @param tar_rt   目标位置偏差矩阵（工具坐标系）
     * @return  true:目标位置偏差小于0.1
     */
    bool isJointReached(QVector<double> tarpos);

    /**
     * @brief 是否达到目标关节位置
     * @param tar_rt   目标位置偏差矩阵（工具坐标系）
     * @return  true:目标位置偏差小于0.1
     */
    bool isEndReached(QVector<double> tarpos);

    /**
     * @brief 获取LINK状态信息
     * @param tar_rt   目标位置偏差矩阵（工具坐标系）
     * @return  目标位置（基坐标系）
     */

    /** end 外部接口函数************************/

private:
    // 日志相关，【后续从这里分离出去，单独封装到一个类里】
    std::map<E_LinkCommd, std::string> enumToString_LinkCommd = {
        {E_LinkCommd::eLINK_NONE, "eLINK_NONE"},
        {E_LinkCommd::eLINK_RESET, "eLINK_RESET"},
        {E_LinkCommd::eLINK_HOME, "eLINK_HOME"},
        {E_LinkCommd::eLINK_POWER, "eLINK_POWER"},
        {E_LinkCommd::eLINK_STOP, "eLINK_STOP"},
        {E_LinkCommd::eLINK_HALT, "eLINK_HALT"},
        {E_LinkCommd::eLINK_MOV, "eLINK_MOV"},
    };
    // 函数，根据枚举值返回对应的字符串
    std::string GetEnumName_LinkCommd(E_LinkCommd value);

    std::map<E_LinkState, std::string> enumToString_LinkState = {
        {E_LinkState::eLINK_ERRORSTOP, "eLINK_ERRORSTOP"},
        {E_LinkState::eLINK_DISABLED, "eLINK_DISABLED"},
        {E_LinkState::eLINK_STANDSTILL, "eLINK_STANDSTILL"},
        {E_LinkState::eLINK_STOPPING, "eLINK_STOPPING"},
        {E_LinkState::eLINK_HOMING, "eLINK_HOMING"},
        {E_LinkState::eLINK_MOVING, "eLINK_MOVING"},
    };
    // 函数，根据枚举值返回对应的字符串
    std::string GetEnumName_LinkState(E_LinkState value);
};

#endif // ROBOT_H
