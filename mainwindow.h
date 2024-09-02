    /*****************************************************************
* 函数名称： mainwindow
* 功能描述： ui交互及主线程操作
******************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <QTime>
#include <QThread>
#include <QDebug>
#include <QString>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QLineEdit>
#include <QByteArray>
#include <QDir>
#include <QtMath>
#include <cmath>
#include <QMutex>
//--------------机器人部分--------------//
#include "com/ComInterface.h"
#include "robot/robot.h"
#include "GVL.h"
//--------------视觉部分--------------//
#include "vision/VisionControls.h"
#include "vision/VisionInterface.h"

//--------------任务部分--------------//
#include "Task/Task.h"
#include <QAtomicInt>

//--------------可视化----------------//
#include "vision/CameraDisplay.h"
#include <bitset>

//--------------日志文件----------------//
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include "GVL.h"




QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE




class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


    // 日志文件初始化
    void initLog();
    std::shared_ptr<spdlog::logger> logger;

    //引用机器人接口类实例
    ComInterface* m_Com;
    //机器人
    CRobot* m_Robot;
    //任务
    CTask* m_Task;

    //视觉模块
    VisionControls* m_Vision;
    VisionInterface* m_VisionInterface;

    //控十字激光
    QByteArray m_CrossLidar;
    // 手动操作限制  待修改
    const double axisVelLimit[JointNum][2]={
            {0,7}, //0 底部升降
            {0,7}, //1 前后平台
            {0,7}, //2 左右平台
            {0,4},  //3 腰旋转
            {0,4},  //4 大臂  待修正
            {0,4},  //5 腕部  待修正
            {0,8},  //6 末端升降
    };
    bool getLineStatus();
    void setLineStatus(bool lineStatus);
    QMutex m_mutex;
    QAtomicInt      buttionIndex; // 按钮索引原子变量记录




    // 操作按钮索引配置
    std::map<std::string, unsigned int> m_btnIndex = {
        //作业操作模块
      {"btn_location_",     0},   // 位置准备
      {"btn_lift_",         1},   // 举升
      {"btn_leveling_",     2},   //调平
      {"btn_lift_2",        3},   //举升对边
      {"btn_sideline_",     4},   //对齐边线
      {"btn_magnet_open_",  5},   //开启磁铁
      {"btn_auto_knock_",   6},   //自动碰钉
      {"btn_magnet_close_", 7},  //关闭磁铁
      {"btn_magnet_pause_", 8},  //碰钉暂停
      {"btn_knock_suspend_", 9}, // 碰钉终止
      {"btn_magnet_stop_", 10},  // 停止
      {"btn_magent_crash_stop_", 11},  // 急停
      // 顶部操作模块
      {"btn_enable_", 12}, // 上使能
      {"btn_disable_", 13}, // 下使能
      {"btn_cleanError_", 14}, // 清除错误
      {"btn_errorStop_", 15}, // 紧急停止
      {"btn_exit", 16}, // 退出

      // 单轴操作
      {"btn_moveFwd_shaft0", 17},
      {"btn_moveFwd_shaft1", 18},
      {"btn_moveFwd_shaft2", 19},
      {"btn_moveFwd_shaft3", 20},
      {"btn_moveFwd_shaft4", 21},
      {"btn_moveFwd_shaft5", 22},
      {"btn_moveFwd_shaft6", 23},
      {"btn_moveFwd_shaft7", 24},
      {"btn_moveFwd_shaft8", 25},
      {"btn_moveFwd_shaft9", 26},
      {"btn_moveFwd_shaft10", 27},

      {"btn_moveBwd_shaft0", 28},
      {"btn_moveBwd_shaft1", 29},
      {"btn_moveBwd_shaft2", 30},
      {"btn_moveBwd_shaft3", 31},
      {"btn_moveBwd_shaft4", 32},
      {"btn_moveBwd_shaft5", 33},
      {"btn_moveBwd_shaft6", 34},
      {"btn_moveBwd_shaft7", 35},
      {"btn_moveBwd_shaft8", 36},
      {"btn_moveBwd_shaft9", 37},
      {"btn_moveBwd_shaft10", 38},

      {"btn_moveRel_shaft0", 39},
      {"btn_moveRel_shaft1", 40},
      {"btn_moveRel_shaft2", 41},
      {"btn_moveRel_shaft3", 42},
      {"btn_moveRel_shaft4", 43},
      {"btn_moveRel_shaft5", 44},
      {"btn_moveRel_shaft6", 45},
      {"btn_moveRel_shaft7", 46},
      {"btn_moveRel_shaft8", 47},
      {"btn_moveRel_shaft9", 48},
      {"btn_moveRel_shaft10", 49},

      // 末端操作
      {"btn_moveFwd_end0", 50},
      {"btn_moveFwd_end1", 51},
      {"btn_moveFwd_end2", 52},
      {"btn_moveFwd_end3", 53},
      {"btn_moveFwd_end4", 54},
      {"btn_moveFwd_end5", 55},

      {"btn_moveBwd_end0", 56},
      {"btn_moveBwd_end1", 57},
      {"btn_moveBwd_end2", 58},
      {"btn_moveBwd_end3", 59},
      {"btn_moveBwd_end4", 60},
      {"btn_moveBwd_end5", 61},

      {"btn_moveRel_end0", 62},
      {"btn_moveRel_end1", 63},
      {"btn_moveRel_end2", 64},
      {"btn_moveRel_end3", 65},
      {"btn_moveRel_end4", 66},
      {"btn_moveRel_end5", 67}
    };

    // 工作作业按钮名称-索引值
    std::map<std::string, unsigned int> m_jobBtnIndex = {
      {"btn_leveling_",      0},          // 调平
      {"btn_sideline_",      1},          // 对齐边线
      {"btn_magnet_open_",   2},          // 吸合
      {"btn_auto_knock_",    3},          // 碰钉
      {"btn_magnet_close_",  4},          // 脱开
      {"btn_magnet_exit_",   5},          // 退出
      {"btn_magnet_pause_",  6},          // 暂停
      {"btn_knock_suspend_", 7},          // 终止
      {"btn_lift_",          8},          // 举升
      {"btn_add_nail",       9},          // 放钉
      {"btn_magnet_stop_",  10},          // 停止
      {"btn_magent_crash_stop_", 11}      // 急停
    };





private:
    Ui::MainWindow *ui;
    void initUiForm();

    QTimer* updateUiTimer;

    const unsigned int larserNum=4;
    const unsigned int cameraNum=6;
    const unsigned int jointNum=10;
    const unsigned int freeJointNum=6;

    void updateCameraData();//更新相机数据
    void updateLaserData();//更新点激光数值
    void updateActionSta();//更新任务流程的状态
    void updateAxisStatus();//更新单轴连接状态
    void updateConnectSta();//更新硬件连接状态
    void setButtonIndex();  // 记录当前触发按钮索引
    void setActionIndex();  // 记录当前触发动作索引(工作流程记录)
    void closeEvent(QCloseEvent *event);
    bool lineStatus = false; // 是否处于直线检测状态


private slots:
    //1.0
    void on_btn_enable_clicked();
    void on_btn_disable_clicked();
    void on_btn_setRobotReset_clicked();
    void on_btn_setLinkHalt_clicked();
    void on_btn_developerMode_clicked();
    void on_btn_userMode_clicked();


    //2.0
    void on_btn_openCamera_clicked();
    void on_btn_closeCamera_clicked();
    void on_btn_location_clicked();
    void on_btn_lift_clicked();
    void on_btn_lift_2clicked();
    void on_btn_leveling_clicked();
    void on_btn_sideline_clicked();
    void on_btn_magnet_open_clicked();
    void on_btn_auto_knock_clicked();
    void on_btn_magnet_close_clicked();
    void on_btn_magnet_pause_clicked();
    void on_btn_knock_suspend_clicked();
    void on_btn_magnet_stop_clicked();
    void on_btn_magent_crash_stop_clicked();
    void on_btn_SetTools_clicked();
    void on_btn_SetMagent_clicked();
    void slots_on_btn_magnet_exit_clicked();
    void slots_on_btn_add_nail_clicked();
    //3.0 update the UI
    void slotUpdateUIAll();

    //4.0
    void btn_moveFwd_shaft_pressed();
    void btn_moveFwd_shaft_released();
    void btn_moveBwd_shaft_pressed();
    void btn_moveBwd_shaft_released();
    void on_moveRel_shaft_clicked();

    void btn_moveFwd_end_pressed();
    void btn_moveFwd_end_released();
    void btn_moveBwd_end_pressed();
    void btn_moveBwd_end_released();
    void on_moveRel_end_clicked();

    // 5.0
    void on_btn_line_detect_clicked();
    void on_btn_line_detect_debug_clicked();
    void on_btn_camera_capture_clicked();
    void on_btn_camera_save_clicked();


    void on_comboBox_tools_currentIndexChanged();
    void on_comboBox_magents_currentIndexChanged();


    void updateLineDetectResults();
    void connectSlotFunctions();


    void initUiWiget();

    void updataDeviceConnectState();
};






#endif // MAINWINDOW_H
