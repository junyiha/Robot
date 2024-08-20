#ifndef CTASK_H
#define CTASK_H

#include "../robot/robot.h"
#include "../com/ComInterface.h"
#include "../com/Manual.h"
#include <QAtomicInt>
#include <QLineEdit>
#include <spdlog/spdlog.h>
#include "../vision/VisionInterface.h"


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
    stMeasureData getStMeasureData();//传感器状态值 接口

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

};

#endif // CTASK_H
