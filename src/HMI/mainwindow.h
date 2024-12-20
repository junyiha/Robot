/*****************************************************************
* 函数名称： mainwindow
* 功能描述： ui交互及主线程操作
******************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
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
#include <QMutex>
#include <QAtomicInt>
#include <QRegularExpression>

#include <iostream>
#include <cmath>
#include <bitset>
#include <thread>
//--------------日志文件----------------//
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <QMouseEvent>
//--------------机器人部分--------------//
#include "com/ComInterface.h"
#include "robot/robot.h"
#include "GVL.h"
//--------------视觉部分--------------//
#include "vision/VisionInterface.h"

//--------------任务部分--------------//
#include "task/Task.h"
#include "ConfigManager.hpp"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private:
    void InitLog();
    void InitUiForm();
    void InitVision();
    void InitSlotUpdateUIAllTimer();
    void InitSlotUpdateAllDevicesTimer();
    void closeEvent(QCloseEvent* event);
    void connectSlotFunctions();

private:
    bool getLineStatus();
    void setLineStatus(bool lineStatus);
    void setButtonIndex();  // 记录当前触发按钮索引
    void setActionIndex();  // 记录当前触发动作索引(工作流程记录)
    void slotUpdateUIAll();

private:
    void updateLineDetectResults();
    void updataDeviceConnectState();
    void updateCameraData();//更新相机数据
    void updateLaserData();//更新点激光数值
    void updateAxisStatus();//更新单轴连接状态
    void updateConnectSta();//更新硬件连接状态
    void updateWorkdScenario();
    void updateConfigurationView();
    void updateTaskStateMachineStatus();
    void slotUpdateImagesAndOtherTimeConsuming();

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

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
    void slots_on_btn_magnet_exit_clicked();
    void slots_on_btn_add_nail_clicked();
    void slots_on_btn_auto_laminate_clicked();// 自动贴合

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

    void slots_btn_laser_upper_enable_clicked();
    void slots_btn_laser_lower_enable_clicked();

    // 6.0 舵轮移动
    void on_btn_wheel_forward_pressed();
    void on_btn_wheel_forward_released();
    void on_btn_wheel_backward_pressed();
    void on_btn_wheel_backward_released();
    void on_btn_wheel_left_pressed();
    void on_btn_wheel_left_released();
    void on_btn_wheel_right_pressed();
    void on_btn_wheel_right_released();
    void on_btn_steering_left_pressed();
    void on_btn_steering_left_released();
    void on_btn_steering_right_pressed();
    void on_btn_steering_right_released();
    void on_btn_add_nail_pressed();
    void on_btn_add_nail_released();
    void on_btn_preparation_pos_pressed();
    void on_btn_preparation_pos_released();

    // 灯光
    void slots_on_btn_camera_width_light_clicked();
    void slots_on_btn_camera_height_light_clicked();
    void slots_btn_camera_hole_light_clicked();
    void slots_btn_laser_light_clicked();
    void slots_on_btn_putter_forward_pressed();
    void slots_on_btn_putter_forward_released();
    void slots_on_btn_putter_backward_pressed();
    void slots_on_btn_putter_backward_released();
    // 参数配置文件
    void slots_btn_load_configuration_clicked();
    void slots_btn_save_prepare_position_clicked();
    void slots_btn_save_lift_position_clicked();
    void slots_on_line_results_dis_clicked();
    void clearFlowButtonStyle();

    void slots_btn_joint_test_clicked();
    void slots_btn_move_zero_clicked();
    void slots_btn_move_zero_select_all_clicked();
    void slots_btn_joint_text_terminate_clicked();

    void slots_btn_global_exit_clicked();

private:
    double m_wheelVel{ 30.0 };
    bool m_dragEnabled;
    const unsigned int larserNum = 4;
    const unsigned int cameraNum = 6;
    const unsigned int jointNum = 11;
    const unsigned int freeJointNum = 6;
    bool lineStatus = false; // 是否处于直线检测状态
    bool lineCameraLightWidthEnable = true;
    bool lineCameraLightHeightEnable = true;
    bool holeCameraLightEnable = true;
    bool laserLightEnable = true;
    bool laserOnUpperEnable = false;
    bool laserOnLowerEnable = false;

    Ui::MainWindow* ui;
    QByteArray m_CrossLidar;
    QMutex m_mutex;
    QTimer* updateUiTimer;
    QAtomicInt      buttionIndex; // 按钮索引原子变量记录
    QTimer* updateCameraTimer;

    ComInterface* m_Com;
    CRobot* m_Robot;
    CTask* m_Task;
    VisionInterface* m_VisionInterface;
    std::unique_ptr<Config::ConfigManager> m_config_ptr;
    std::shared_ptr<spdlog::logger> logger;

    bool m_thread_exit_flag{ false };
    std::vector<std::thread*> m_thread_pool;

    // 操作按钮索引配置
    std::map<std::string, unsigned int> m_btnIndex = {
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
            {"btn_leveling_",      1},          // 调平
            {"btn_sideline_",      2},          // 对齐边线
            {"btn_magnet_open_",   3},          // 吸合
            {"btn_auto_knock_",    4},          // 碰钉
            {"btn_magnet_close_",  5},          // 脱开
            {"btn_magnet_exit_",   6},          // 退出
            {"btn_magnet_pause_",  7},          // 暂停
            {"btn_knock_suspend_", 8},          // 终止
            {"btn_preparation_pos",9},         // 举升
            {"btn_add_nail",       10},          // 放钉
            {"btn_magnet_stop_",  11},          // 停止
            {"btn_magent_crash_stop_", 12}      // 急停

    };
};

#endif // MAINWINDOW_H