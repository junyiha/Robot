/*****************************************************************
* 函数名称： ComInterface
* 功能描述： ·基于RobotCom类和IOCom类，建立机器人对象和io对象的统一接口
*          ·使用单例模式，全局唯一且易获取类的引用，方便跨线程访问
* 参数说明： 参数说明
* 返回值：   返回值说明
******************************************************************/
#ifndef COMINTERFACE_H
#define COMINTERFACE_H

#include <QObject>
#include <QThread>
#include <QByteArray>
#include <Eigen/Dense>
#include "RobotCom.h"
#include "Tools.h"
#include "Manual.h"
#include "../GVL.h"
#include "../Task/Measure.h"
#include "BoardingTool.h"
#include <spdlog/spdlog.h>
#include <QMutex>
#include "LaserDistanceBojke.h"

class ComInterface : public QThread
{
    Q_OBJECT
public:
    //饿汉式单例模式
    static ComInterface* getInstance();
    ComInterface(const ComInterface&) = delete;
    ComInterface& operator = (const ComInterface&) = delete;
    explicit ComInterface(QObject* parent = nullptr);
    ~ComInterface();
    void closeThread();
    CManual     m_cManual;

protected:
    //设备成员
    RobotCom	m_cRobot;
    CTools		m_cTools;
    BoardingTool m_cToolsBoarding;
    LaserDistanceBojke m_cToolsBoardingLaser;
    std::shared_ptr<spdlog::logger> log;

    void run();



private:

    int isRobotConnected = false;// 0表示成功,-1失败
    int isIOConnected = false;
    bool running = true;
    QMutex mutex_read;
public:
    //写操作用信号，读操作可以跨线程直接读，所以这里只需要控制指令映射
    stLinkCommand	m_main_LinkCommd[MAX_FREEDOM_LINK];
    st_SetAxis		m_main_AxisCommd[MAX_FREEDOM_ROBOT];

    /**
     *@brief 获取机器人连接状态
     *@return true 已连接，false 未连接
     */
    bool getRobotConnectSta();

    /**
     *@brief 获取IO板连接状态
     * @param index IO板索引
     * @return true 已连接，false 未连接
     */
    bool getIOConnectSta(const uint index);


    /**
     * @brief setHoleLightStatus 控制对孔相机补光灯开关
     * @param flag
     */
    void setHoleLightStatus(const bool flag);
    /**
     * @brief setCrossLightStatus 控制十字激光开关
     */
    void setCrossLightStatus(const bool flag);
    /**
     * @brief getLasersStatus 获取点激光的测距数据
     * @return 长度为5的double向量，前四个为高精度点激光,[4]为距离均值，[5],最大偏差
     */
     //QVector<double> getLasersDistance();


    QVector<double> getLasersDistanceBoardingByBojke();

    QVector<double> getLasersDistanceBoarding();
    /**
     * @brief getJointGroupStatus 读所有轴组数据
     * @return 轴组状态向量
     */
    QVector<st_ReadAxis> getJointGroupStatus();

    /**
    * @brief 轴组关节运动
    * @param index: LINK索引
    * @param pos:   目标位置
    * @param vel:   设定速度
    * @return
    */
    void setLinkJointMoveAbs(uint index, double pos[], double vel[]);

    /**
    * @brief 轴组关节速度
    * @param index: LINK索引
    * @param pos:  目标位置
    * @param vel:  设定速度
    * @return
    */
    void setLinkJointMoveVel(uint index, double vel[]);

    /**
    * @brief 获取link轴组关节状态
    * @param index: LINK索引
    * @return QVector<st_ReadAxis> 轴组状态向量
    */
    QVector<st_ReadAxis> getLinkJointStatus(uint index);

    /**
    * @brief 获取link状态
    * @param index: LINK索引
    * @return QVector<st_ReadAxis> 轴组状态向量
    */
    StatusofLink getLinkStatus(uint index);

    /**
     * @brief 获取遥控器指令
     *
     * @param m_Manual: 遥控器指令数据
     */
    void getManual(stManualCmd& m_Manual);
    void getManual(stManualOperator& m_Manual);
    //xxxxxxxxx未定义

    //*************机器人控制相关接口信号*************

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
    void RobotMove(E_MotionMode mode, double pos[], double vel[], uint freedom);

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
     * @brief 多轴实时运动控制函数 高频发送运动控制命令接口 单轴
     * @param joint_index 轴索引
     * @param mode 运动模式
     * @param pos 位置
     * @param vel 速度
     * @param end_vel 结束速度
     * @param distance 方向
     * @return [不确定]
     */
    bool setJointGroupMove(eMC_Motion mode, const double pos[] = NULL, const double vel[] = NULL, double end_vel[] = NULL, double distance[] = NULL);



    //*************IO控制相关接口信号*************
    /**
     * @brief SetIOA 配置IO板A
     * @param value 配置参数
     */
    void SetIOA(QByteArray value);


    /**
     * @brief getCommState_Robot 获取机器人连接状态
     * @return 已连接则true
     */
    bool getCommState_Robot() { return m_cRobot.getCommState(); }

    /**
     * @brief getCommState_IOA 获取IOA板子连接状态
     * @return 已连接则true
     */
    bool getCommState_IOA() { return m_cToolsBoarding.m_cIOA.getCommState(); }
    bool getCommState_IOB() { return m_cTools.m_cIOB.getCommState(); }

    /**
     * @brief 激光控制
     *
     * @param swit 开关量
     */
    void setCrossLaser(bool swit);

    /**
     * @brief 工具控制
     * @param  index 工具编号1~10
     * @param  action 动作类型
     * @return
     */
    void SetToolsAction(quint8 index, E_WeldAction action);
    /**
     * @brief 磁铁控制
     * @param  index 磁铁编号1,2,3,4; 0表示所有磁铁
     * @return action 动作类型
     */
    void SetMagentAction(quint8 index = 0, E_MagentAction action = eNONE_Magent);

    /**
     * @brief 设置焊枪连接
     * @param index 焊枪编号，0表示关闭所有
     */
    void SetGunConnect(qint8 index);

    /**
    * @brief  灯光控制
    * @param  index 灯光编号0，全部 1.长边，2.短边，3.孔位
    * @param  On    开关量
    */
    void SetLight(quint8 index, bool On)
    {
        m_cToolsBoarding.SetLight(index, On);
    }


    /**
   * @brief  激光标线控制
   * @param  On    开关量
   */
    void SetLaserMarker(bool On)
    {
        m_cToolsBoarding.SetLaserMarker(On);
    }

    /**
    * @brief  推杆控制
    * @param  push  -1 推荐倒推，0 停止， 1推荐正推
    **/
    void SetCylinder(int push)
    {
        m_cToolsBoarding.SetCylinder(push);
    }
};

#endif // COMINTERFACE_H
