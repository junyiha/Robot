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

private:
    Ui::MainWindow *ui;
    void initUiForm();

    QTimer* updateUiTimer;

    const unsigned int larserNum=4;
    const unsigned int cameraNum=6;
    const unsigned int jointNum=7;
    const unsigned int freeJointNum=6;

    void updateCameraData();//更新相机数据
    void updateLaserData();//更新点激光数值
    void updateActionSta();//更新任务流程的状态
    void updateAxisStatus();//更新单轴连接状态
    void updateConnectSta();//更新硬件连接状态
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

};






#endif // MAINWINDOW_H
