#ifndef CTASK_H
#define CTASK_H

#include <map>
#include <mutex>
#include "PDTask.h"


#include "../robot/robot.h"
#include "../com/ComInterface.h"
#include "../com/Manual.h"
#include <QAtomicInt>
#include <QLineEdit>
#include <spdlog/spdlog.h>
#include "../vision/VisionInterface.h"

enum class ETopState
{
    eManual = 0,        // 手动
    eParallel,          // 调平
    ePositioning,       // 定位
    eReadToMagentOn,    // 待吸合
    eDoWeld,            // 碰钉
    eQuit               // 退出
};

enum class ESubState
{
    eNULL = 0,          // 空状态
    eNotReady = 0,      // 未就绪
    eReady,             // 就绪
    eMotion,            // 运动

    eReadyToParallel,   // 待调平
    eDetection,         // 检测

    eReadyToPositioning,// 待定位
    
    eReadyToDoWeld,     // 待碰钉
    eDoingWeld,         // 碰钉中
    eStopWeld,          // 碰钉停止

    eQuiting,           // 退出中
    ePause              // 暂停
};

enum class EExecutionCommand
{
    eNULL = 0,          // 空指令
    eManual = 0,        // 手动指令
    eParallel,          // 调平
    eTerminate,         // 终止
    ePause,             // 暂停
    ePositioning,       // 定位
    eMagentOn,          // 吸合
    eQuit,              // 退出
    eAutoWeld,          // 自动碰钉
    eMagentOff,         // 脱开
    eStopWeld           // 停止碰钉
};

enum class EDetectionInParallelResult
{
    eDeviationIsLessThanThreshold = 0,     // 激光传感器偏差小于阈值
    eDistanceMeetsRequirement,             // 板壁距离满足调整要求
    eNoWallDetected                        // 未检测到壁面
};

enum class EDetectionInPositioningResult
{
    eDeviationIsLessThanThreshold = 0,     // 边线偏差小于阈值
    eEndAdjustmentDataIsValid,             // 末端调整数据合法
    eDataIsInvalid,                         // 数据非法

};

/////////////////////////////////////////////////////////////////

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
const double LINE_DEVIATION_THRESHOLD = 1.0;//边线调整允许偏差

 //碰钉动作序列，及动作周期数(50ms)，根据实际工艺调整；eWeld_Up eWeld_Down绑定了接触器，必须保留
const QVector<E_WeldAction> ActionList ={eGrind_MovorOff, eGrind_OnorDown, eGrind_Up, eGrind_OnorDown, eGrind_MovorOff, eWeld_MovorDwon, eWeld_Fix, eWeld_Up, eWeld_On, eWeld_Down, eInitAction};
const QVector<int>          ActionTime ={             40,              20,       150,              20,              20,              40,        40,       40,       40,         40,          5};
const QVector<std::string>  ActionName ={"GrindMovorOff","Grind_OnorDown","Grind_Up","Weld_MovorDwon","Grind_MovorOff","Weld_MovorDwon","Weld_Fix","Weld_Up","Weld_On","Weld_Down","InitAction"};

class CTask:public QThread
{
    Q_OBJECT
public:
    explicit CTask(ComInterface* comm,CRobot* robot, VisionInterface* vision, QObject *parent = nullptr);

    QAtomicInt      ActionIndex;//半自动、按钮测试用
    /**
     * @brief 获取传感器状态数据
     * 
     * @return stMeasureData 传感器状态数据
     */
    stMeasureData getStMeasureData();//传感器状态值 接口
    /**
     * @brief 结束任务线程
     * 
     */
    void closeThread();

    bool         m_bMagnetOn;        //磁铁吸合状态

protected:
    ComInterface*   m_Comm = NULL;
    CRobot*         m_Robot = NULL;
    VisionInterface* m_vision = NULL;

    QLineEdit ** lineEdit_endRelMove;

    bool             m_AutoWorking; //自动模式状态标志位，内部逻辑判断
    AutoProcessStage m_Step;        //自动作业阶段状态，内部逻辑判断

    stLinkStatus     m_LinkStatus;               //机器人状态状态
    QVector<st_ReadAxis> m_JointGroupStatus;         //轴组状态
    QVector<Eigen::Matrix4d> m_TargetDeviation;  //目标位姿（工具系下）
    stManualCmd    m_Manual;       //遥控器指令
    stManualCmd    m_preManual;    //上一帧遥控器指令

    stMeasureData m_stMeasuredata; //传感器状态反馈
    stMeasureData _stMeasuredata;  //传感器状态反馈


    //  机器人

    bool         c_running = true;


    std::shared_ptr<spdlog::logger> log;


    QMutex          mutex_cmd;
    QMutex          mutex_read;
    QMutex          mutex_write;

    /**
    * @brief 运行函数
    */

    void run() override;

    /**
    * @brief 更新机器人状态和外部指令
    */
    void updateCmdandStatus();

    /**
    * @brief 半自动作业过程
    */
    void SemiAutoProgrcess();

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

    //bool  DoAction(int stage);

    /**
     * @brief 磁铁下降
     * @return
     */
    bool doMagentDown();
    /**
     * @brief PrintTargetPos 利用qDebug()打印矩阵(替代cout)
     * @param m_TargetDeviation
     */
    void PrintTargetPos(uint index,QVector<Eigen::Matrix4d> m_TargetDeviation);

    /**
     * 日志打印
     */
    //日志相关，【后续从这里分离出去，单独封装到一个类里】
    std::map<int, std::string> enumToString_ActionIndex= {
            //=====================0~10 :空闲、急停、按暂停 等 ==========================
            {0, "[ 无----NULL ]"},
            {1, "[ 急停----Halt]"},
            {3, "[ 停止----Stop]"},
            //=====================10~19 :举升调平 ==========================
            {10, "[ 举升到准备位置----ready ]"},
            {12, "[ 举升到调平位置----ready to parallel]"},
            {15, "[ 调平----parallel]"},
            {16, "[ 举升到对边位置----ready to line]"},
            //=====================20~29:边线对齐=====================
            {20, "[ 计算边线调整量----cal line adjustment]"},
            {25, "[ 进行边线调整----line adjust]"},
            {28, "[ 检测调整结果----check adjust result]"},
            //=====================30~50:碰钉动作=====================
            {30, "[ 磁铁吸合----Magnet On ]"},
            {32, "[ 磁铁脱合----Magnet Off ]"},
            {40, "[ 自动碰钉----Weld ]"},
            {42, "[ 自动碰钉暂停----Weld Pause ]"},
            {44, "[ 结束/中止碰钉----Weld End ]"},
    };
    // 函数，根据枚举值返回对应的字符串
    std::string GetEnumName_action_index(const int value)
    {
        auto it = enumToString_ActionIndex.find(value);
        if (it != enumToString_ActionIndex.end()) {
            return it->second;
        } else {
            throw std::runtime_error("Unknown enum value");
        }
    }

///////////////////////////////////////////--0827新增函数--//////////////////////////////////////////////////////
private:
    /**
     * @brief 调平检测函数，根据激光测距数据判断是否具备调平条件或完成调平
     * @param laserDistance[4] 激光测距数据
     * @return  -1:不具备调平条件， 0,可执行调平， 1:完成调平
    */
    int CheckParallelState(QVector<double>  laserDistance);
    EDetectionInParallelResult CheckParallelStateDecorator(QVector<double>  laserDistance);

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

public:
    /**
     * @brief 更新第一层和第二层状态(线程安全)
     * 
     * @param topState 
     * @param subState 
     */
    void updateTopAndSubState(ETopState topState, ESubState subState);

    /**
     * @brief 更新执行指令(线程安全)
     * 
     * @param executionCommand 
     */
    void updateExecutionCommand(EExecutionCommand executionCommand = EExecutionCommand::eNULL);

    /**
     * @brief 获取当前状态字符串，格式: 第一层状态--第二层状态
     * 
     * @return std::string 
     */
    std::string getCurrentStateString();

    /**
     * @brief 获取当前执行指令字符串
     * 
     * @return std::string 
     */
    std::string getCurrentExecutionCommandString();

private:
    ETopState m_etopState{ETopState::eManual};
    ESubState m_esubState{ESubState::eReady};
    EExecutionCommand m_eexecutionCommand{EExecutionCommand::eNULL};
    
private:
    std::mutex m_mutex;
    std::shared_ptr<TASK::PDTask> m_pdTaskPtr;

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
};

#endif // CTASK_H
