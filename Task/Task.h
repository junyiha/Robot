#ifndef CTASK_H
#define CTASK_H

#include <map>
#include <mutex>

#include "../robot/robot.h"
#include "../com/ComInterface.h"
#include "../com/Manual.h"
#include <QAtomicInt>
#include <QLineEdit>
#include <spdlog/spdlog.h>
#include "../vision/VisionInterface.h"
#include "TaskExternal.h"

enum AutoProcessStage
{
    eEnd= 0,
    eA  = 1,
    eB1 = 2,
    eB2 = 3,
    eC1 = 4,
    eC2 = 5,
    eD1 = 6,
    eD2 = 7,
};

enum ECommadforTask
{
    eNONE = 0,
    eStart = 1,
    eCLOSE = 2,
    eReset = 3,
    eStop = 4

};

const double PARRALLE_DISTANCE = 5.0; //调平允许偏差
const double LINE_DEVIATION_THRESHOLD = 2.0;//边线调整允许偏差  1.0

// 装板机器人
const double LINE_DEVIATION_BOARDING_THRESHOLD = 5.0;//贴合允许偏差
const int que_size = 7;
const std::vector<double> BOARDING_MOTION_QUE = { 60,30,20,15,10,5,0 }; //贴合运动序列

 //碰钉动作序列，及动作周期数(50ms)，根据实际工艺调整；eWeld_Up eWeld_Down绑定了接触器，必须保留
const QVector<E_WeldAction> ActionList ={eGrind_MovorOff, eGrind_OnorDown, eGrind_Up, eGrind_OnorDown, eGrind_MovorOff, eWeld_MovorDwon, eWeld_Fix, eWeld_Up, eWeld_On, eWeld_Down, eInitAction};
const QVector<int>          ActionTime ={             40,              20,       100,              20,              20,              40,        40,       40,       40,         40,          5};
const QVector<std::string>  ActionName ={"GrindMovorOff","Grind_OnorDown","Grind_Up","Weld_MovorDwon","Grind_MovorOff","Weld_MovorDwon","Weld_Fix","Weld_Up","Weld_On","Weld_Down","InitAction"};

class CTask:public QThread
{
    Q_OBJECT
public:
    explicit CTask(ComInterface* comm,CRobot* robot, VisionInterface* vision, QObject *parent = nullptr);

    /**
     * @brief 传感器状态值.
     */
    stMeasureData getStMeasureData();

    /**
     * @brief 结束任务线程
     * 
     */
    void closeThread();

public:
    bool m_bMagnetOn;  //磁铁吸合状态
    QAtomicInt ActionIndex;  //半自动、按钮测试用
    QAtomicInt ButtonIndex;  //当前点击按钮索引

protected:
    CRobot* m_Robot{nullptr};
    ComInterface* m_Comm{ nullptr };
    VisionInterface* m_vision{nullptr};

    stLinkStatus     m_LinkStatus;               //机器人状态状态
    QVector<st_ReadAxis> m_JointGroupStatus;         //轴组状态
    QVector<Eigen::Matrix4d> m_TargetDeviation;  //目标位姿（工具系下）
    stManualCmd    m_Manual;       //遥控器指令
    stManualCmd    m_preManual;    //上一帧遥控器指令

    stManualOperator m_manualOperator;
    stManualOperator m_preManualOperator;

    stMeasureData m_stMeasuredata; //传感器状态反馈
    stMeasureData _stMeasuredata;  //传感器状态反馈

    //  机器人
    QMutex mutex_read;
    bool c_running{ true };
    std::shared_ptr<spdlog::logger> log;

protected:
    /**
    * @brief 运行函数
    */

    void run() override;

    /**
    * @brief 更新机器人状态和外部指令
    */
    void updateCmdandStatus();

    /**
    * @brief 自动作业过程
    */
    //void AutoProgrcess();

    /**
    * @brief 手动操作指令处理
    */
    void Manual();

    /**
    * @brief 自动碰钉函数
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
    EDetectionInParallelResult CheckParallelStateDecorator(QVector<double>  laserDistance);
    EDetectionInParallelResult CheckParallelStateDecorator(double laserDistance[]);

    /**
     * @brief 定位检测函数，根据相机返回数据，判断是否具备定位条件或完成定位
     * @param /   内部调用传感器参数m_stMeasuredata
     * @return  -1:不具备定位条件， 0,可执行定位， 1:完成定位
     */
    int CheckPositionState();
    EDetectionInPositioningResult CheckPositionStateDecorator();

    /**
     * @brief 定位检测函数，根据相机返回数据，判断是否具备定位条件或完成定位
     * @param /   内部调用传感器参数m_stMeasuredata
     * @param  motion_index 运动序列索引
     * @return  -1:不具备定位条件， 0,可执行定位， 1:完成定位
     */
    int CheckBoardingState(uint& motion_index);
    EDetectionInPositioningResult CheckBoardingStateDecorator();

    int CheckBoardingStateForPositioning();
    EDetectionInPositioningResult CheckBoardingStateForPositioningDecorator();
    int CheckBoardingStateForFitBoard(uint& motion_index);
    EDetectionInFitBoardResult CheckBoardingStateForFitBoardDecorator();

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
     * @brief  贴合状态
     * 
     */
    void fitBoardStateTransition();

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
     * @brief 贴合--待贴合状态下，可执行指令
     * 
     */
    void readyToFitBoardExecutionCommand();

    /**
     * @brief 贴合--检测状态下，可执行指令
     * 
     */
    void detectionInFitBoardExecutionCommand();

    /**
     * @brief 贴合--运动状态下，可执行指令
     * 
     */
    void motionInFitBoardExecutionCommand();

    /**
     * @brief 贴合--贴合完成状态下，可执行指令
     * 
     */
    void fitBoardFinishedExecutionCommand();

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
     * @brief 贴合状态下计算调整量
     * 
     */
    void CalculatedAdjustmentForFitBoard();
       
    /**
     * @brief 更新雷达数据.
     */
    void UpdateLaserDistance();

    /**
     * @brief 更新视觉数据.
     */
    void UpdateVisionResult(VisionResult &vis_res);

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

private:
    /**
     * @brief 将遥控器的命令转换为内部指令
     * 
     */
    void TranslateManualTaskIndexNumberToCMD();

private:
    ETopState m_etopState{ETopState::eManual};
    ESubState m_esubState{ESubState::eNotReady};
    EExecutionCommand m_eexecutionCommand{EExecutionCommand::eNULL};
    
private:
    std::mutex m_mutex;
    bool m_position_motion_flag{false};
    uint m_motion_index{ 0 };
    std::vector<double> m_fit_board_target_pose{0, 0, 0, 0, 0, 0};

    std::map<ETopState, std::pair<std::string, std::string>> TopStateStringMap 
    {
        {ETopState::eManual, {"手动", "Manual"}},
        {ETopState::eParallel, {"调平", "Parallel"}},
        {ETopState::ePositioning, {"定位", "Positioning"}},
        {ETopState::eFitBoard, {"贴合", "FitBoard"}},
        {ETopState::eReadToMagentOn, {"待吸合", "ReadToMagentOn"}},
        {ETopState::eDoWeld, {"碰钉", "DoWeld"}},
        {ETopState::eQuit, {"退出", "Quit"}}
    };
    std::map<ESubState, std::pair<std::string, std::string>> SubStateStringMap
    {
        {ESubState::eNULL, {"空状态", "NULL"}},
        {ESubState::eNotReady, {"未就绪", "NotReady"}},
        {ESubState::eReady, {"就绪", "Ready"}},
        {ESubState::eMotion, {"运动", "Motion"}},
        {ESubState::eReadyToParallel, {"待调平", "ReadyToParallel"}},
        {ESubState::eDetection, {"检测", "Detection"}},
        {ESubState::eReadyToFitBoard, {"待贴合", "ReadyToFitBoard"}},
        {ESubState::eFitBoardFinished, {"贴合完成", "FitBoardFinished"}},
        {ESubState::eReadyToPositioning, {"待定位", "ReadyToPositioning"}},
        {ESubState::eReadyToDoWeld, {"待碰钉", "ReadyToDoWeld"}},
        {ESubState::eDoingWeld, {"碰钉中", "DoingWeld"}},
        {ESubState::eStopWeld, {"碰钉停止", "StopWeld"}},
        {ESubState::eQuiting, {"退出中", "Quiting"}},
        {ESubState::ePause, {"暂停", "Pause"}}
    };
    std::map<EExecutionCommand, std::string> ExecutionCommandStringMap
    {
        {EExecutionCommand::eNULL, "空指令"},
        {EExecutionCommand::eManual, "手动指令"},
        {EExecutionCommand::eParallel, "调平"},
        {EExecutionCommand::eTerminate, "终止"},
        {EExecutionCommand::ePause, "暂停"},
        {EExecutionCommand::ePositioning, "定位"},
        {EExecutionCommand::eFitBoard, "贴合"},
        {EExecutionCommand::eMagentOn, "吸合"},
        {EExecutionCommand::eQuit, "退出"},
        {EExecutionCommand::eAutoWeld, "自动碰钉"},
        {EExecutionCommand::eMagentOff, "脱开"},
        {EExecutionCommand::eStopWeld, "停止碰钉"}
    };
};

#endif // CTASK_H