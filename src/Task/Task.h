#ifndef CTASK_H
#define CTASK_H

#include <QAtomicInt>
#include <QLineEdit>

#include "utils/basic_header.hpp"
#include "vision/VisionInterface.h"
#include "task/TaskExternal.h"
#include "robot/robot.h"
#include "com/ComInterface.h"

class CTask : public QThread
{
    Q_OBJECT
public:
    explicit CTask(ComInterface* comm, CRobot* robot, VisionInterface* vision, QObject* parent = nullptr);

    /**
     * @brief 结束任务线程
     *
     */
    void closeThread();

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

    bool SingleSideLine();

    bool SingleParallel();

    void SecondPush();
    void SecondQuit();

    void SetVisionReplaceFlag(int index, bool flag);
    bool GetVisionReplaceFlag(int index);

public:
    QAtomicInt ActionIndex; // 半自动、按钮测试用
    QAtomicInt ButtonIndex; // 当前点击按钮索引

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
     * @brief 手动操作指令处理
     */
    void Manual();

private:
    /**
     * @brief 调平检测函数，根据激光测距数据判断是否具备调平条件或完成调平
     * @param laserDistance[4] 激光测距数据
     * @return  -1:不具备调平条件， 0,可执行调平， 1:完成调平
     */
    int CheckParallelState(std::vector<double> laserDistance, int max_deviation, int min_deviation, int lift_distance);
    /**
     * @brief 调平检测函数装饰器.
     */
    EDetectionInParallelResult CheckParallelStateDecorator();

    /**
     * @brief 对边检测函数，根据相机返回数据，判断是否具备定位条件或完成定位
     * @param /   内部调用传感器参数m_stMeasuredata
     * @param  motion_index 运动序列索引
     * @return  -1:不具备对边条件， 0,可执行对边， 1:完成对边
     */
    int CheckSidelineState();
    /**
     * @brief 对边检测函数装饰器.
     */
    EDetectionInPositioningResult CheckSidelineStateDecorator();

    /**
     * @brief 检查是否贴合完成.
     */
    EDetectionInFitBoardResult CheckFitBoardState();

    /**
     * @brief 终止函数，停止自动过程，切换到手动模式
     */
    void TaskTerminate();

    ///////////////////////////////////////////--状态机部分--//////////////////////////////////////////////////////
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
     * @brief  贴合状态
     *
     */
    void fitBoardStateTransition();

    /**
     * @brief 退出状态
     *
     */
    void quitStateTransition();

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
     * @brief 贴合--对边运动状态下，可执行指令.
     */
    void sidelineMotionInFitBoardExecutionCommand();

    /**
     * @brief 贴合--举升运动状态下，可执行指令.
     */
    void liftMotionInFitBoardExecutionCommand();

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

    /**
     * @brief 贴合状态下计算对边运动的调整量
     *
     */
    void CalculatedAdjustmentOfSideline();

    /**
     * @brief 贴合状态下计算举升运动的调整量.
     */
    void CalculatedAdjustmentOfLift();

    /**
     * @brief 更新雷达数据.
     */
    void UpdateLaserDistance();

    /**
     * @brief 更新视觉数据.
     */
    void UpdateVisionResult(VisionResult& vis_res);

    /**
     * @brief 将遥控器的命令转换为内部指令
     *
     */
    void TranslateManualTaskIndexNumberToCMD();

    void SteerWheelControl();

private:
    ETopState m_etopState{ ETopState::eManual };
    ESubState m_esubState{ ESubState::eNotReady };
    EExecutionCommand m_eexecutionCommand{ EExecutionCommand::eNULL };

protected:
    CRobot* m_Robot{ nullptr };
    ComInterface* m_Comm{ nullptr };
    VisionInterface* m_vision{ nullptr };
    stLinkStatus m_LinkStatus;                  // 机器人状态状态
    QVector<st_ReadAxis> m_JointGroupStatus;    // 轴组状态
    stManualOperator m_manualOperator;
    stManualOperator m_preManualOperator;
    stMeasureData m_stMeasuredata; // 传感器状态反馈

    //  机器人
    bool c_running{ true };
    std::shared_ptr<spdlog::logger> log;

private:
    std::atomic<bool> m_single_job_flag{ false };
    std::mutex m_mutex;
    bool m_position_motion_flag{ false };
    uint m_motion_index{ 0 };
    std::vector<double> m_fit_board_target_pose{ 0, 0, 0, 0, 0, 0 };
    double m_lift_tool{ 0.0 };
    const std::vector<double> BOARDING_MOTION_QUE = { 75, 70, 60, 55, 50, 45, 35, 8, 3, 0 }; // 贴合运动序列
    std::vector<std::thread*> m_thread_pool;
    std::map<int, bool> m_vision_replace{
        { 1, false },
        { 3, false },
        { 4, false },
        { 5, false }
    };

    std::map<ETopState, std::pair<std::string, std::string>> TopStateStringMap{
        {ETopState::eManual, {"手动", "Manual"}},
        {ETopState::eParallel, {"调平", "Parallel"}},
        {ETopState::ePositioning, {"定位", "Positioning"}},
        {ETopState::eFitBoard, {"贴合", "FitBoard"}},
        {ETopState::eQuit, {"退出", "Quit"}} };
    std::map<ESubState, std::pair<std::string, std::string>> SubStateStringMap{
        {ESubState::eNULL, {"空状态", "NULL"}},
        {ESubState::eNotReady, {"未就绪", "NotReady"}},
        {ESubState::eReady, {"就绪", "Ready"}},
        {ESubState::eMotion, {"运动", "Motion"}},
        {ESubState::eReadyToParallel, {"待调平", "ReadyToParallel"}},
        {ESubState::eDetection, {"检测", "Detection"}},
        {ESubState::eReadyToFitBoard, {"待贴合", "ReadyToFitBoard"}},
        {ESubState::eSidelineMotion, {"对边运动", "SidelineMotion"}},
        {ESubState::eLiftMotion, {"举升运动", "LiftMotion"}},
        {ESubState::eFitBoardFinished, {"贴合完成", "FitBoardFinished"}},
        {ESubState::eReadyToPositioning, {"待定位", "ReadyToPositioning"}},
        {ESubState::eQuiting, {"退出中", "Quiting"}},
        {ESubState::ePause, {"暂停", "Pause"}} };
    std::map<EExecutionCommand, std::string> ExecutionCommandStringMap{
        {EExecutionCommand::eNULL, "空指令"},
        {EExecutionCommand::eManual, "手动指令"},
        {EExecutionCommand::eParallel, "调平"},
        {EExecutionCommand::eTerminate, "终止"},
        {EExecutionCommand::ePause, "暂停"},
        {EExecutionCommand::ePositioning, "定位"},
        {EExecutionCommand::eFitBoard, "贴合"},
        {EExecutionCommand::eQuit, "退出"}
    };
};

#endif // CTASK_H