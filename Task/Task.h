#ifndef CTASK_H
#define CTASK_H

#include <map>
#include <mutex>
#include <iomanip>
#include <bitset>
#include <functional>
#include <fstream>

#include "../robot/robot.h"
#include "../com/ComInterface.h"
#include "../com/Manual.h"
#include <QAtomicInt>
#include <QLineEdit>
#include <spdlog/spdlog.h>
#include "../vision/VisionInterface.h"
#include "TaskExternal.hpp"

//碰钉动作序列，及动作周期数(50ms)，根据实际工艺调整；eWeld_Up eWeld_Down绑定了接触器，必须保留
const QVector<E_WeldAction> ActionList = { eGrind_MovorOff, eGrind_OnorDown, eGrind_Up, eGrind_OnorDown, eGrind_MovorOff, eWeld_MovorDwon, eWeld_Fix, eWeld_Up, eWeld_On, eWeld_Down, eInitAction };
const QVector<int>          ActionTime = { 40,              20,       100,              20,              20,              40,        40,       40,       40,         40,          5 };
const QVector<std::string>  ActionName = { "GrindMovorOff","Grind_OnorDown","Grind_Up","Weld_MovorDwon","Grind_MovorOff","Weld_MovorDwon","Weld_Fix","Weld_Up","Weld_On","Weld_Down","InitAction" };

class CTask :public QThread
{
    Q_OBJECT
public:
    explicit CTask(ComInterface* comm, CRobot* robot, VisionInterface* vision, QObject* parent = nullptr);

    QAtomicInt      ActionIndex;//半自动、按钮测试用
    QAtomicInt      ButtonIndex; //当前点击按钮索引
    /**
     * @brief 结束任务线程
     *
     */
    void closeThread();

    bool m_bMagnetOn{ false };        //磁铁吸合状态

protected:
    ComInterface* m_Comm = NULL;
    CRobot* m_Robot = NULL;
    VisionInterface* m_vision = NULL;

    stLinkStatus     m_LinkStatus;               //机器人状态状态
    QVector<st_ReadAxis> m_JointGroupStatus;         //轴组状态

    stManualOperator m_manualOperator;
    stManualOperator m_preManualOperator;

    stMeasureData m_stMeasuredata; //传感器状态反馈
    stMeasureData _stMeasuredata;  //传感器状态反馈

    //  机器人
    bool         c_running = true;
    std::shared_ptr<spdlog::logger> log;
    QMutex          mutex_read;

    /**
    * @brief 运行函数
    */

    void run() override;

    /**
    * @brief 更新机器人状态和外部指令
    */
    void updateCmdandStatus();

    /**
    * @brief 手动操作指令处理
    */
    void Manual();

    /**
     * @brief 舵轮控制.
     */
    void SteerWheelControl();

    /**
     * @brief 行走轮控制.
     */
    void TravelWheelControl();

    /**
     * @brief 机器人运动.
     */
    void RobotMotion();

    /**
     * @brief 遥控器指令转换为内部命令.
     */
    void ManualIndexToCommand();

    /**
    * @brief 自动碰钉函数(旧版本)
    * @param execute   -1:结束，0:暂停， 1：执行
    */
    bool doWeldAction(qint8 execute);

    /**
    * @brief 吸合磁铁
    */
    bool doMagentOn();

    /**
    * @brief 磁体相关
    */
    bool doMagentOff();

    ///////////////////////////////////////////--0827新增函数--//////////////////////////////////////////////////////
private:
    /**
     * @brief 调平检测函数，根据激光测距数据判断是否具备调平条件或完成调平
     * @param laserDistance[4] 激光测距数据
     * @return  -1:不具备调平条件， 0,可执行调平， 1:完成调平
    */
    int CheckParallelState(QVector<double>  laserDistance);
    EDetectionInParallelResult CheckParallelStateDecorator();

    /**
     * @brief 定位检测函数，根据相机返回数据，判断是否具备定位条件或完成定位
     * @param /   内部调用传感器参数m_stMeasuredata
     * @return  -1:不具备定位条件， 0,可执行定位， 1:完成定位
     */
    int CheckPositionState();
    EDetectionInPositioningResult CheckPositionStateDecorator();

    /**
     * @brief 终止函数，停止自动过程，切换到手动模式
     */
    void TaskTerminate();

    ///////////////////////////////////////////--状态机部分--//////////////////////////////////////////////////////
private:
    /**
     * @brief 状态转换函数
     *
     */
    void stateTransition();

    /**
     * @brief 手动状态
     *
     */
    void manualStateTransition();

    /**
     * @brief 调平状态
     *
     */
    void parallelStateTransition();

    /**
     * @brief 定位状态
     *
     */
    void positioningStateTransition();

    /**
     * @brief 待吸合状态
     *
     */
    void readyToMagentOnStateTransition();

    /**
     * @brief 碰钉状态
     *
     */
    void doWeldStateTransition();

    /**
     * @brief 退出状态
     *
     */
    void quitStateTransition();

private:
    /**
     * @brief 手动指令
     *
     */
    void readyExecutionCommand();

    /**
     * @brief 未就绪状态下执行的手动指令
     *
     */
    void notReadyExecutionCommand();

    /**
     * @brief 调平--待调平状态下，可执行指令
     *
     */
    void readyToParallelExecutionCommand();

    /**
     * @brief 调平--检测状态下，可执行指令
     *
     */
    void detectionInParallelExecutionCommand();

    /**
     * @brief 定位--检测状态下，可执行指令
     *
     */
    void detectionInPositioningExecutionCommand();

    /**
     * @brief 调平--运动状态下，可执行指令
     *
     */
    void motionInParallelExecutionCommand();

    /**
     * @brief 定位--运动状态下，可执行指令
     *
     */
    void motionInPositioningExecutionCommand();

    /**
     * @brief 定位--待定位状态下，可执行指令
     *
     */
    void readyToPositioningExecutionCommand();

    /**
     * @brief 待吸合状态下，可执行指令
     *
     */
    void readyToMagentOnExecutionCommand();

    /**
     * @brief 碰钉--待碰钉状态下，可执行指令
     *
     */
    void readyToWeldExecutionCommand();

    /**
     * @brief 碰钉--碰钉中状态下，可执行指令
     *
     */
    void doingWeldExecutionCommand();

    /**
     * @brief 碰钉--停止状态下，可执行指令
     *
     */
    void stopWeldExecutionCommand();

    /**
     * @brief 退出--退出中状态下，可执行指令
     *
     */
    void quitingExecutionCommand();

    /**
     * @brief 退出--暂停状态下，可执行指令
     *
     */
    void pauseExecutionCommand();

    /**
     * @brief 终止指令。停止运行，状态跳转至: 手动
     *
     */
    void terminateCommand();

private:
    /**
     * @brief 更新雷达数据.
     */
    void UpdateLaserDistance();

    /**
     * @brief 更新视觉数据.
     */
    void UpdateVisionResult(VisionResult& vis_res);

    /**
     * @brief 两个碰钉工具执行单元.
     */
    void DoubleToolsDoWeldingExecuteUnit(int tool_a, int tool_b, int key);

    /**
     * @brief 新的碰钉流程，两组(四个工具)交叉执行.
     *
     * @param [execute] -1, 停止碰钉。0，暂停碰钉。1，开始碰钉。
     *
     * @return bool
     */
    bool FourToolsDoWeldAction(int execute);

    /**
     * @brief 备用方案，两组(两个工具)交叉执行，避免两个打磨同时进行的问题.
     */
    bool DoubleToolsDoWeldAction(int execute);

public:
    /**
     * @brief 更新第一层和第二层状态(线程安全)
     *
     * @param topState
     * @param subState
     */
    void updateTopAndSubState(ETopState topState, ESubState subState);

    /**
     * @brief 更新执行指令(线程安全 默认更新为空指令)
     *
     * @param executionCommand
     */
    void updateExecutionCommand(EExecutionCommand executionCommand = EExecutionCommand::eNULL);

    /**
     * @brief 获取当前状态字符串，格式: 第一层状态--第二层状态
     *
     * @return std::string
     */
    std::string getCurrentStateString() const;

    /**
     * @brief 获取当前执行指令字符串
     *
     * @return std::string
     */
    std::string getCurrentExecutionCommandString() const;

    /**
     * @brief 检测第二层状态.
     *
     * @param subState
     *
     * @return true | false
     */
    bool checkSubState(ESubState subState) const;

    /**
     * @brief 将遥控器的命令转换为内部指令
     *
     */
    void TranslateManualTaskIndexNumberToCMD();

    /**
     * @brief 脱开磁铁.
     */
    bool DoMagentOff();

    /**
     * @brief 自动碰钉.
     */
    bool DoWeldActionDecorator(int execute);

    /**
     * @brief 获取当前工作模式.
     * @return bool: true, 自动化 | false, 半自动化.
     */
    bool GetWorkingMode();

    /**
     * @brief 设置工作模式，默认为半自动化。
     * @param mode: true, 自动化 | false, 半自动化.
     */
    void SetWorkdingMode(const bool mode);

    /**
     * @brief 设置回调函数，用于调用消息提示框.
     */
    void SetCallBack(std::function<void(const std::string&)> callback);

    /**
     * @brief 检查点激光数据是否满足作业条件
     */
    bool LaserValueIsValid();

    /**
    * @brief 检查线间距数据是否满足作业条件
    */
    bool LineValueIsValid();

    /**
     * @brief 一个碰钉工具执行单元.
     */
    void SingleToolDoWeldingExecuteUnit(int index, int key);

private:
    ETopState m_etopState{ ETopState::eManual };
    ESubState m_esubState{ ESubState::eReady };
    EExecutionCommand m_eexecutionCommand{ EExecutionCommand::eNULL };

private:
    std::mutex m_mutex;
    bool m_position_motion_flag{ false };
    bool m_automatic_working_flag{ true };
    std::map<std::string, int> ValueMap;
    std::function<void(const std::string&)> m_callback;
    std::chrono::time_point<std::chrono::system_clock> m_begin_time;
    std::ofstream m_timer_record;

    std::map<ETopState, std::string> TopStateStringMap
    {
        {ETopState::eManual, "手动"},
        {ETopState::eParallel, "调平"},
        {ETopState::ePositioning, "定位"},
        {ETopState::eReadToMagentOn, "待吸合"},
        {ETopState::eDoWeld, "碰钉"},
        {ETopState::eQuit, "退出"}
    };
    std::map<ESubState, std::string> SubStateStringMap
    {
        {ESubState::eNULL, "空状态"},
        {ESubState::eNotReady, "未就绪"},
        {ESubState::eReady, "就绪"},
        {ESubState::eMotion, "运动"},
        {ESubState::eReadyToParallel, "待调平"},
        {ESubState::eDetection, "检测"},
        {ESubState::eReadyToPositioning, "待定位"},
        {ESubState::eReadyToDoWeld, "待碰钉"},
        {ESubState::eDoingWeld, "碰钉中"},
        {ESubState::eStopWeld, "碰钉停止"},
        {ESubState::eQuiting, "退出中"},
        {ESubState::ePause, "暂停"}
    };
    std::map<EExecutionCommand, std::string> ExecutionCommandStringMap
    {
        {EExecutionCommand::eNULL, "空指令"},
        {EExecutionCommand::eManual, "手动指令"},
        {EExecutionCommand::eParallel, "调平"},
        {EExecutionCommand::eTerminate, "终止"},
        {EExecutionCommand::ePause, "暂停"},
        {EExecutionCommand::ePositioning, "定位"},
        {EExecutionCommand::eMagentOn, "吸合"},
        {EExecutionCommand::eQuit, "退出"},
        {EExecutionCommand::eAutoWeld, "自动碰钉"},
        {EExecutionCommand::eMagentOff, "脱开"},
        {EExecutionCommand::eStopWeld, "停止碰钉"}
    };

    std::map<ActionKey, std::string> ActionMap =
    {
        { ActionKey::Grind_MovorOff1, "Grind_MovorOff1 " },
        { ActionKey::Grind_OnorDown1, "Grind_OnorDown1 " },
        { ActionKey::Grind_Up, "Grind_Up " },
        { ActionKey::Grind_OnorDown2, "Grind_OnorDown2 " },
        { ActionKey::Grind_MovorOff2, "Grind_MovorOff2 " },
        { ActionKey::Weld_MovorDwon, "Weld_MovorDwon " },
        { ActionKey::Weld_Fix, "Weld_Fix " },
        { ActionKey::Weld_Up, "Weld_Up " },
        { ActionKey::Weld_On, "Weld_On " },
        { ActionKey::Weld_Down, "Weld_Down " },
        { ActionKey::InitAction, "InitAction " }
    };
};

#endif // CTASK_H