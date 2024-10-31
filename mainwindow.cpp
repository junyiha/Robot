#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //1.0 初始化化日志
    initLog();


    //2.0 UI界面初始化
    initUiForm();

    //3.0 DO 初始化
    this->logger->trace("初始化DO映射控制字");
    for(int i=0;i<DOBYTENUM;i++)
    {
        qint8 temp = 0;
        m_CrossLidar.append(temp);
    }

    //4.0 初始化机器人、视觉、任务
    m_Com = ComInterface::getInstance();
    m_Robot = new CRobot(m_Com);
    m_VisionInterface = new VisionInterface();
    m_Task = new CTask(m_Com,m_Robot,m_VisionInterface);
    m_config_ptr = std::make_unique<Config::ConfigManager>();

    //5.0 计时器界面实时更新 (100ms更新一次)
    this->logger->trace("启动界面实时更新");
    this->updateUiTimer = new QTimer(this);
    updateUiTimer->setInterval(10);
    connect(updateUiTimer,&QTimer::timeout,this,&MainWindow::slotUpdateUIAll);

    // 6.0 直线检测专用定时器


    //7.0 启动线程
    this->logger->trace("启动功能对象...");
    m_Com->start();
    m_Robot->start();
    m_VisionInterface->start();
    m_Task->start();
    updateUiTimer->start();

}

void MainWindow::initUiForm() {

    // 1.0 初始化UI控件
    initUiWiget();

    // 2.0 为按钮绑定操函数
    connectSlotFunctions();


//    // 单轴
//    QString posPrefix = "";
//    QString moveFwdPrefix = "";
//    QString moveBwdPrefix = "";
//    QString moveRelPrefix = "";
//
//    // 按钮时间绑定
////    for(unsigned int i = 0; i<6;i++){
//////        connect(&findChild(posPrefix + QString::number(i)), &QPushButton::clicked, this, &MainWindow::on_btn_enable_yaofuyang_clicked );
////
////    }
}

void MainWindow::initUiWiget() {//2.0 碰钉工具测试用-----------------------------------
    for(int i=0;i<10;i++)
    {
        ui->comboBox_tools_->addItem(tr("tool ") + QString::number(i + 1));
    }
    ui->comboBox_magents_->addItem(tr("all mangents "));
    for(int i=0;i<4;i++)
    {
        ui->comboBox_magents_->addItem(tr("mangent ") + QString::number(i + 1));
    }
    QStringList QList;
    QList<<tr("eInitAction")
         <<tr("eGrind_MovorOff")
         <<tr("eGrind_OnorDown")
         <<tr("eGrind_Up")
         <<tr("eWeld_MovorDwon")
         <<tr("eWeld_Fix")
         <<tr("eWeld_Up")
         <<tr("eWeld_On")
         <<tr("eWeld_Down")
         <<tr("eNone_Action");
    ui->comboBox_tools_action->addItems(QList);
    QList.clear();
    QList<<tr("eNONE_Magent")<<tr("eMag_On")<<tr("eMag_Off")<<tr("eMag_Up")<<tr("eMag_Down");
    ui->comboBox_magents_action->addItems(QList);
}

void MainWindow::connectSlotFunctions() {// 按钮时间绑定

     // 1.0 工具栏功能按钮槽函数绑定
    connect(ui->btn_enable_, &QPushButton::clicked, this, &MainWindow::on_btn_enable_clicked, Qt::UniqueConnection);  // 上使能
    connect(ui->btn_disable_, &QPushButton::clicked, this, &MainWindow::on_btn_disable_clicked, Qt::UniqueConnection);// 下使能
    connect(ui->btn_cleanError_, &QPushButton::clicked, this, &MainWindow::on_btn_setRobotReset_clicked, Qt::UniqueConnection);// 清除错误
    connect(ui->btn_errorStop_, &QPushButton::clicked, this, &MainWindow::on_btn_setLinkHalt_clicked, Qt::UniqueConnection);// 紧急停止
    connect(ui->btn_developerMode_, &QPushButton::clicked, this, &MainWindow::on_btn_developerMode_clicked, Qt::UniqueConnection);// 开发者模式
    connect(ui->btn_userMode_, &QPushButton::clicked, this, &MainWindow::on_btn_userMode_clicked, Qt::UniqueConnection);// 用户模式

    // 2.0 流程操作栏按钮槽函数绑定
//    connect(ui->btn_camera_open, &QPushButton::clicked, this, &MainWindow::on_btn_openCamera_clicked, Qt::UniqueConnection); // 打开摄像头
//    connect(ui->btn_camera_close, &QPushButton::clicked, this, &MainWindow::on_btn_closeCamera_clicked, Qt::UniqueConnection);
//    connect(ui->btn_location_, &QPushButton::clicked, this, &MainWindow::on_btn_location_clicked, Qt::UniqueConnection);
//    connect(ui->btn_lift_, &QPushButton::clicked, this, &MainWindow::on_btn_lift_clicked, Qt::UniqueConnection);
    connect(ui->btn_leveling_, &QPushButton::clicked, this, &MainWindow::on_btn_leveling_clicked, Qt::UniqueConnection);
    connect(ui->btn_sideline_, &QPushButton::clicked, this, &MainWindow::on_btn_sideline_clicked, Qt::UniqueConnection);
    connect(ui->btn_magnet_open_, &QPushButton::clicked, this, &MainWindow::on_btn_magnet_open_clicked, Qt::UniqueConnection);
    connect(ui->btn_auto_knock_, &QPushButton::clicked, this, &MainWindow::on_btn_auto_knock_clicked, Qt::UniqueConnection);
    connect(ui->btn_magnet_close_, &QPushButton::clicked, this, &MainWindow::on_btn_magnet_close_clicked, Qt::UniqueConnection);
    connect(ui->btn_magnet_pause_, &QPushButton::clicked, this, &MainWindow::on_btn_magnet_pause_clicked, Qt::UniqueConnection);
    connect(ui->btn_knock_suspend_, &QPushButton::clicked, this, &MainWindow::on_btn_knock_suspend_clicked, Qt::UniqueConnection);
    connect(ui->btn_magnet_stop_, &QPushButton::clicked, this, &MainWindow::on_btn_magnet_stop_clicked, Qt::UniqueConnection);
    connect(ui->btn_magent_crash_stop_, &QPushButton::clicked, this, &MainWindow::on_btn_magent_crash_stop_clicked, Qt::UniqueConnection);
    connect(ui->btn_magnet_exit, &QPushButton::clicked, this, &MainWindow::slots_on_btn_magnet_exit_clicked, Qt::UniqueConnection);
    connect(ui->btn_add_nail, &QPushButton::clicked, this, &MainWindow::slots_on_btn_add_nail_clicked, Qt::UniqueConnection);

    //用于调试。临时加的btn_lift_2
//    connect(ui->btn_lift_2, &QPushButton::clicked, this, &MainWindow::on_btn_lift_2clicked, Qt::UniqueConnection);
    // 3.0 机械臂功能按钮槽函数绑定
    for(unsigned int i = 0; i < jointNum; i++){

        // 单轴
        connect(findChild<QPushButton*>("btn_moveFwd_shaft" + QString::number(i)), &QPushButton::pressed, this, &MainWindow::btn_moveFwd_shaft_pressed, Qt::UniqueConnection);
        connect(findChild<QPushButton*>("btn_moveFwd_shaft" + QString::number(i)), &QPushButton::released, this, &MainWindow::btn_moveFwd_shaft_released, Qt::UniqueConnection);
        connect(findChild<QPushButton*>("btn_moveBwd_shaft" + QString::number(i)), &QPushButton::pressed, this, &MainWindow::btn_moveBwd_shaft_pressed, Qt::UniqueConnection);
        connect(findChild<QPushButton*>("btn_moveBwd_shaft" + QString::number(i)), &QPushButton::released, this, &MainWindow::btn_moveBwd_shaft_released, Qt::UniqueConnection);
        connect(findChild<QPushButton*>("btn_moveRel_shaft" + QString::number(i)), &QPushButton::clicked, this, &MainWindow::on_moveRel_shaft_clicked, Qt::UniqueConnection);
    }

    for(unsigned int i = 0; i < freeJointNum; i++){
        // 末端联动
        connect(findChild<QPushButton*>("btn_moveFwd_end" + QString::number(i)), &QPushButton::pressed, this, &MainWindow::btn_moveFwd_end_pressed, Qt::UniqueConnection);
        connect(findChild<QPushButton*>("btn_moveFwd_end" + QString::number(i)), &QPushButton::released, this, &MainWindow::btn_moveFwd_end_released, Qt::UniqueConnection);
        connect(findChild<QPushButton*>("btn_moveBwd_end" + QString::number(i)), &QPushButton::pressed, this, &MainWindow::btn_moveBwd_end_pressed, Qt::UniqueConnection);
        connect(findChild<QPushButton*>("btn_moveBwd_end" + QString::number(i)), &QPushButton::released, this, &MainWindow::btn_moveBwd_end_released, Qt::UniqueConnection);
        connect(findChild<QPushButton*>("btn_moveRel_end" + QString::number(i)), &QPushButton::clicked, this, &MainWindow::on_moveRel_end_clicked, Qt::UniqueConnection);
    }

    // 4.0 碰钉下拉框按钮绑定
    connect(ui->comboBox_tools_, &QComboBox::currentTextChanged, this, &MainWindow::on_comboBox_tools_currentIndexChanged, Qt::UniqueConnection);
    connect(ui->comboBox_magents_, &QComboBox::currentTextChanged, this, &MainWindow::on_comboBox_magents_currentIndexChanged, Qt::UniqueConnection);


    // 5.0 视觉可视化区域按钮绑定
    connect(ui->btn_line_detect, &QPushButton::clicked, this, &MainWindow::on_btn_line_detect_clicked, Qt::UniqueConnection);
    connect(ui->btn_line_detect_debug, &QPushButton::clicked, this, &MainWindow::on_btn_line_detect_debug_clicked, Qt::UniqueConnection);
    connect(ui->btn_camera_capture, &QPushButton::clicked, this, &MainWindow::on_btn_camera_capture_clicked, Qt::UniqueConnection);
    connect(ui->btn_savePicture, &QPushButton::clicked, this, &MainWindow::on_btn_camera_save_clicked, Qt::UniqueConnection);

    // 底盘移动
    connect(ui->btn_wheel_forward, &QPushButton::pressed, this, &MainWindow::on_btn_wheel_forward_pressed, Qt::UniqueConnection);
    connect(ui->btn_wheel_forward, &QPushButton::released, this, &MainWindow::on_btn_wheel_forward_released, Qt::UniqueConnection);

    connect(ui->btn_wheel_backward, &QPushButton::pressed, this, &MainWindow::on_btn_wheel_backward_pressed, Qt::UniqueConnection);
    connect(ui->btn_wheel_backward, &QPushButton::released, this, &MainWindow::on_btn_wheel_backward_released, Qt::UniqueConnection);

    connect(ui->btn_wheel_left, &QPushButton::pressed, this, &MainWindow::on_btn_wheel_left_pressed, Qt::UniqueConnection);
    connect(ui->btn_wheel_left, &QPushButton::released, this, &MainWindow::on_btn_wheel_left_released, Qt::UniqueConnection);

    connect(ui->btn_wheel_right, &QPushButton::pressed, this, &MainWindow::on_btn_wheel_right_pressed, Qt::UniqueConnection);
    connect(ui->btn_wheel_right, &QPushButton::released, this, &MainWindow::on_btn_wheel_right_released, Qt::UniqueConnection);

    // 舵轮移动
    connect(ui->btn_steering_left, &QPushButton::pressed, this, &MainWindow::on_btn_steering_left_pressed, Qt::UniqueConnection);
    connect(ui->btn_steering_left, &QPushButton::released, this, &MainWindow::on_btn_steering_left_released, Qt::UniqueConnection);
    connect(ui->btn_steering_right, &QPushButton::pressed, this, &MainWindow::on_btn_steering_right_pressed, Qt::UniqueConnection);
    connect(ui->btn_steering_right, &QPushButton::released, this, &MainWindow::on_btn_steering_right_released, Qt::UniqueConnection);

    // 添加钉子
    connect(ui->btn_add_nail, &QPushButton::pressed, this, &MainWindow::on_btn_add_nail_pressed, Qt::UniqueConnection);
    connect(ui->btn_add_nail, &QPushButton::released, this, &MainWindow::on_btn_add_nail_released, Qt::UniqueConnection);

    //准备位置
    connect(ui->btn_preparation_pos, &QPushButton::pressed, this, &MainWindow::on_btn_preparation_pos_pressed, Qt::UniqueConnection);
    connect(ui->btn_preparation_pos, &QPushButton::released, this, &MainWindow::on_btn_preparation_pos_released, Qt::UniqueConnection);

    // 参数配置文件
    connect(ui->btn_load_configuration, &QPushButton::clicked, this, &MainWindow::slots_btn_load_configuration_clicked, Qt::UniqueConnection);
    connect(ui->btn_save_home_position, &QPushButton::clicked, this, &MainWindow::slots_btn_save_home_position_clicked, Qt::UniqueConnection);
    connect(ui->btn_save_prepare_position, &QPushButton::clicked, this, &MainWindow::slots_btn_save_prepare_position_clicked, Qt::UniqueConnection);
    connect(ui->btn_save_quit_position, &QPushButton::clicked, this, &MainWindow::slots_btn_save_quit_position_clicked, Qt::UniqueConnection);
}

MainWindow::~MainWindow()
{


}

void MainWindow::initLog()
{
    //创建控制台日志记录器
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
//    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S:%e] [%^%l%$] %v");

    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e][thread %t][%@,%!] [%l] : %v");

    // 创建文件日志记录器: 滚动记录，最大文件5M，文件数量100个
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("logs/rotating.txt", 1048576 *5 , 100);

    //同步记录器，
    std::vector<spdlog::sink_ptr> sinks {console_sink, rotating_sink};
    auto logger = std::make_shared<spdlog::logger>("logger", sinks.begin(), sinks.end());

    spdlog::register_logger(logger); //注册为全局日志，通过log_write访问;
    spdlog::flush_every(std::chrono::seconds(3)); //每3s刷新一次
    //根据需要调整记录级别：调试debug，发布info
    spdlog::set_level(spdlog::level::debug);
    this->logger = spdlog::get("logger");
}

//上使能函数
void MainWindow::on_btn_enable_clicked()
{
    setButtonIndex();
    //这里使用link层且只有link[0]一个索引
    if(m_Com->getCommState_Robot() == true)
    {
//        m_Robot->setLinkEnable(true);  // 仅机器臂使能
        m_Robot->setRobotEnable(true);
        this->logger->info("执行上使能!");
    }
    else
    {
        this->logger->info("上使能失败, 请先连接机器人");
        QMessageBox::critical(this, "上使能无效", "请先连接机器人");
    }
}

//下使能函数
void MainWindow::on_btn_disable_clicked()
{
    setButtonIndex();
    if(m_Com->getCommState_Robot() == true)
    {
        m_Robot->setLinkEnable(false);
        m_Robot->setRobotEnable(false);
        this->logger->info("执行下使能!");
    }
    else
    {
        this->logger->info("下使能失败, 请先连接机器人");
        QMessageBox::critical(this, "下使能无效", "请先连接机器人");
    }
}

// 清除错误
void MainWindow::on_btn_setRobotReset_clicked()
{
    setButtonIndex();
    m_Robot->setRobotReset();
    this->logger->info("清除错误");
}
//紧急停止
void MainWindow::on_btn_setLinkHalt_clicked()
{
    setButtonIndex();

    m_Task->ActionIndex.storeRelaxed(11);
    m_Robot->setLinkStop();
    this->logger->info("紧急停止");
}

void MainWindow::on_btn_openCamera_clicked() {
    if(this->m_VisionInterface!=nullptr){
        this->m_VisionInterface->camera_controls->openCameraAll();
        QThread::msleep(1000);  // 相机启动需要一定缓冲时间
    }

    if(!this->m_VisionInterface->line_handler->is_running){ // 线程并没有启动
        this->m_VisionInterface->line_handler->is_running = true;
        this->m_VisionInterface->line_handler->start();
    }

    this->logger->info("执行相机打开操作");
//    ui->btn_camera_open->setEnabled(false);
//    ui->btn_camera_close->setEnabled(true);
}

void MainWindow::on_btn_closeCamera_clicked() {
    if(this->m_VisionInterface!=nullptr){
        this->m_VisionInterface->camera_controls->closeCameraAll();
        this->m_VisionInterface->line_handler->is_running = false;
    }
    this->logger->info("执行相机关闭操作");
//    ui->btn_camera_open->setEnabled(true);
//    ui->btn_camera_close->setEnabled(false);
}

void MainWindow::on_btn_location_clicked() {
      // 准备位置
    setButtonIndex();
    m_Task->ActionIndex.storeRelaxed(10);
    this->logger->info("进入准备位置...");
}

void MainWindow::on_btn_lift_clicked() {
    // 举升
    setButtonIndex();
    setActionIndex();
    this->logger->info("准备,末端举升");

}

void MainWindow::on_btn_lift_2clicked() {
    setButtonIndex();
    // 举升
    m_Task->ActionIndex.storeRelaxed(16);
    this->logger->info("lift up to 15mm");
}

void MainWindow::on_btn_leveling_clicked() {
    // 调平
    setButtonIndex();
    setActionIndex();
    this->logger->info("启动调平");
}

void MainWindow::on_btn_sideline_clicked() {
   // 对齐边线
    setButtonIndex();
    setActionIndex();
    this->logger->info(" 启动对齐边线");
}

void MainWindow::on_btn_magnet_open_clicked() {
    // 开启磁铁
    setButtonIndex();
    setActionIndex();
    m_Task->m_bMagnetOn = true;
    this->logger->info("磁铁吸合");
}

void MainWindow::on_btn_auto_knock_clicked() {
   // 自动碰钉
    setButtonIndex();
    setActionIndex();
    this->logger->info("碰钉");
}

void MainWindow::on_btn_magnet_close_clicked() {
    // 关闭磁铁
    setButtonIndex();
    setActionIndex();
    m_Task->m_bMagnetOn = false;
    this->logger->info("磁铁脱开");
}

void MainWindow::on_btn_magnet_pause_clicked() {
   // 碰钉暂停
    setButtonIndex();
    setActionIndex();
    this->logger->info("碰钉暂停 ");
}

void MainWindow::on_btn_knock_suspend_clicked() {
   // 碰钉终止
    setButtonIndex();
    setActionIndex();
    this->logger->info("按钮：碰钉中止");
}

void MainWindow::on_btn_magnet_stop_clicked() {
   // 停止
    setButtonIndex();
    setActionIndex();
    this->logger->info("停止");
}

void MainWindow::on_btn_magent_crash_stop_clicked() {
   // 急停
    setButtonIndex();
    setActionIndex();
    this->logger->info("急停");
}

void MainWindow::slotUpdateUIAll() {

    // 1.0 更新点激光测量值
    updateLaserData();

    // 2.0 更新相机图像帧
    updateCameraData();

    // 3.0 更新边线检测实时测量值
    updateLineDetectResults();

    // 4.0 更新轴状态信息
    updateAxisStatus();

    // 5.0 更新机器人任务流程状态
    updateActionSta();

    //6.0 更新设备连接状态
    updateConnectSta();

    // 7.0 更新硬件设备连接状态，并通过指示灯显示
    updataDeviceConnectState();


}

void MainWindow::updataDeviceConnectState() {
    // 相机连接状态显示
    std::vector<bool> camera_open_sta = m_VisionInterface->camera_controls->getCameraOpenedInfo();
    for(int i=0; i<camera_open_sta.size(); i++){
        if(camera_open_sta[i]){
            findChild<QLabel *>("label_camera_state" + QString::number(i + 1))->setStyleSheet(
                    "image: url(:/img/images/icon_greenLight.png);");
        }else{
            findChild<QLabel *>("label_camera_state" + QString::number(i + 1))->setStyleSheet(
                    "image: url(:/img/images/icon_redLight.png);");
        }
    }
    // 更新io板和机器人连接状态显示
    bool io_A_sta = m_Com->getCommState_IOA();
    bool io_B_sta = m_Com->getCommState_IOB();
    bool robot_sta = m_Com->getCommState_Robot();
//    this->logger->info("ioA:{}, ioB:{}, robot:{}", io_A_sta, io_B_sta, robot_sta);

    if(io_A_sta){
        ui->label_io_A->setStyleSheet("image: url(:/img/images/icon_greenLight.png);");
    }else{
        ui->label_io_A->setStyleSheet("image: url(:/img/images/icon_redLight.png);");
    }

    if(io_B_sta){
        ui->label_io_B->setStyleSheet("image: url(:/img/images/icon_greenLight.png);");
    }else{
        ui->label_io_B->setStyleSheet("image: url(:/img/images/icon_redLight.png);");
    }

    if(robot_sta){
        ui->label_robot_state->setStyleSheet("image: url(:/img/images/icon_greenLight.png);");
    }else{
        ui->label_robot_state->setStyleSheet("image: url(:/img/images/icon_redLight.png);");
    }

    // 更新遥控器连接状态
    bool manual_sta =  m_Com->m_cManual.getConnectState();
    if(manual_sta){
        ui->label_controller_state->setStyleSheet("image: url(:/img/images/icon_greenLight.png);");
    }else{
        ui->label_controller_state->setStyleSheet("image: url(:/img/images/icon_redLight.png);");
    }
}

void MainWindow::updateAxisStatus() {

    static int old_JointStatus[10] = {-100,-100,-100,-100,-100,-100,-100, -100,-100,-100};

    // 更新单轴状态信息
    QVector<st_ReadAxis> stJointstatus = m_Com->getLinkJointStatus(0); //link轴组数据

    if(stJointstatus.size() > 0) {
        for (int i = 0; i < jointNum; i++) {
            //更新单轴坐标
            findChild<QLabel *>("label_pos_shaft" + QString::number(i))->setText(
                    QString::number(stJointstatus[i].Position, 10, 2));

            //更新单轴状态
            if (old_JointStatus[i] != stJointstatus[i].eState) {
                old_JointStatus[i] = stJointstatus[i].eState;
                switch (stJointstatus[i].eState) {
                    case eAxis_ERRORSTOP:
                        findChild<QLabel *>("label_device_state" + QString::number(i))->setStyleSheet(
                                "image: url(:/img/images/icon_redLight.png);"
                                "border:1px solid black;"
                                );
                        break;
                    case eAxis_UNDEFINED:
                        findChild<QLabel *>("label_device_state" + QString::number(i))->setStyleSheet(
                                "image: url(:/img/images/icon_greenLight.png);"
                                "border:1px solid black;"
                                );
                        break;
                    case eAxis_DISABLED:
                        findChild<QLabel *>("label_device_state" + QString::number(i))->setStyleSheet(
                                "image: url(:/img/images/icon_yellowLight.png);"
                                "border:1px solid black;"
                                );
                        break;
                    default:
                        findChild<QLabel *>("label_device_state" + QString::number(i))->setStyleSheet(
                                "image: url(:/img/images/icon_greenLight.png);"
                                "border:1px solid black;"
                                );
                        break;
                }
            }
        }
    }

    // 更新末端姿态信息
    stLinkStatus linkstatus = m_Robot->getLinkSta();

    double  robotActPose[6];
    for(int i=0; i < freeJointNum; ++i){
        if(i>2){
            robotActPose[i] = linkstatus.stLinkActKin.LinkPos[i]*57.3;
        }else
        {
            robotActPose[i] = linkstatus.stLinkActKin.LinkPos[i];
        }
        findChild<QLabel*>("label_pos_end" + QString::number(i))->setText(QString::number(robotActPose[i]));
    }

    // 更新舵轮位置
    QVector<st_ReadAxis> jointStatus = m_Com->getJointGroupStatus();
    findChild<QLabel*>("label_steering_pos")->setText(QString::number(jointStatus[GP::STEER_LEFT_INDEX].Position));


}

void MainWindow::updateLineDetectResults() {

    bool isEnable = m_VisionInterface->camera_controls->camerasIsOpened();
    if(!isEnable){
        this->logger->info("相机未全部开启成功,请检查相机连接！");
        return;
    }
    unsigned pageIndex = ui->stackedWidget_view->currentIndex();
    bool lineStatus = this->getLineStatus();
    if(!lineStatus) { // 实时相机实时检测结果显示
        QString prefix;
        if(pageIndex==0){
            prefix = "label_cam_dist"; // 首页结果展示
        }else if(pageIndex==1){
            prefix = "label_line_dist"; // debug页面结果展示
        }else {
//            this->logger->info("相机页面选择错误");
            return ;
        }
        VisionResult visResult = m_VisionInterface->getVisResult();
        if(visResult.status){
            for(int i = 0; i < cameraNum; i++){
                float dist = visResult.stData.m_LineDistance[i];
                dist = std::isinf(dist) ? 0 : dist;
                findChild<QLabel*>(prefix + QString::number(i))->setText("Dist " + QString::number(i + 1) + ":" + QString::number(dist));
            }
        }


//        std::map<std::string, LineDetectRes> res = m_VisionInterface->getLineRes();
//        for(const auto &item : res){
//            std::string prefix =  item.first;
//            size_t index = prefix.find("_");
//            int number = prefix[index+1]-'0';
//            float dist = item.second.dist;
//            // 更新直线检测结果
//            findChild<QLabel*>("label_cam_dist" + QString::number(number-1))->setText("Dist " + QString::number(number) + ":" + QString::number(dist));
//        }
    }

}

void MainWindow::updateLaserData() {
    QVector  laserdis = m_Com->getLasersDistance();
    if(laserdis.size() > 0){
        for(int i = 0; i < larserNum; i++){
            findChild<QLabel*>("label_laserDist" + QString::number(i))->setText(QString::number(laserdis[i]));
        }
    }else{
        logger->info("获取点激光值为空!");
    }
}

void MainWindow::updateCameraData() {

    // 获取相机是否开启
    bool isEnable = m_VisionInterface->camera_controls->camerasIsOpened();
    if(!isEnable){
//       this->logger->info("相机未全部开启,请检查相机连接！");
       return ;
    }

    unsigned pageIndex = ui->stackedWidget_view->currentIndex();
    bool lineStatus = this->getLineStatus();
    if(!lineStatus){ // 实时相机画面展示
        QString prefix;
        cv::Size imgSize;
        if(pageIndex==0){
            prefix = "label_cam_dis";
            imgSize = cv::Size(200,200);
        }else if(pageIndex==1){
            prefix = "label_camera_dis";
            imgSize = cv::Size(400,280);
        }else{
            this->logger->info("相机页面选择错误");
            return ;
        }

        m_VisionInterface->camera_controls->getImageAll();
        std::map<std::string, cv::Mat> cameraData = m_VisionInterface->camera_controls->getCameraImages();
        if(cameraData.size()>0){
            for(const auto &item:cameraData){
                if(item.second.empty()){
                    logger->info("相机{}数据为空", item.first);
                    continue;
                }
                size_t index = item.first.find("_")+1;
                int number = item.first[index]-'0';
                cv::Mat temp;
                cv::resize(item.second,temp,imgSize);
                QImage img = QImage((uchar*)temp.data, temp.cols, temp.rows, QImage::Format_RGB888);
                findChild<QLabel*>(prefix + QString::number(number - 1))->setPixmap(QPixmap::fromImage(img));
//                this->logger->info("相机{}数据获取成功**************************************", item.first);
            }
        }else{
            logger->info("相机数据获取为空");
            return ;
        }
    }
}

void MainWindow::closeEvent(QCloseEvent *event){

    // 关闭相机进程
    this->m_VisionInterface->closeThread();
    this->logger->info("关闭相机进程 closeThread");

    // 关闭串口
    this->m_Com->closeThread();
    this->logger->info("关闭串口 closeThread");

    //关闭机器人
    this->m_Robot->closeThread();
    this->logger->info("关闭机器进程 closeThread");

    // 关闭任务
    this->m_Task->closeThread();
    this->logger->info("关闭任务进程 closeThread");

    // 关闭计时器线程
    this->updateUiTimer->stop();
    disconnect(this->updateUiTimer);

//    delete this->m_Com;
//    delete this->m_Robot;
//    delete this->m_Task;
//    delete this->updateUiTimer;
//    delete this->m_VisionInterface;

}

void MainWindow::btn_moveFwd_shaft_pressed() {


    QPushButton *pressedButton = qobject_cast<QPushButton*>(sender());
    if (!pressedButton){
        return;
    }
    setButtonIndex();

    // 获取按钮在数组中的索引
    int index = -1;
    for (int i = 0; i < jointNum; i++) {
        if (pressedButton == findChild<QPushButton*>("btn_moveFwd_shaft" + QString::number(i))) {
            index = i;
            break;
        }
    }

    if(index>=0 && index<jointNum)
    {
//        m_Robot->setJointMoveVel(index,0.1 * axisVelLimit[index][1]);
        m_Robot->setJointMoveVel(index,0.5 * LINK_0_JOINT_MAX_VEL[index]);
        this->logger->info("************设置轴{}速度为{}****************", index, 0.5 * LINK_0_JOINT_MAX_VEL[index]);
    }

}

void MainWindow::on_btn_developerMode_clicked() {
    ui->stackedWidget_view->setCurrentIndex(2);
}

void MainWindow::on_btn_userMode_clicked() {
   ui->stackedWidget_view->setCurrentIndex(0);
}

void MainWindow::on_comboBox_tools_currentIndexChanged() {
    this->logger->info("选择工具");
    int index = ui->comboBox_tools_->currentIndex()+1;
    QString action_str = ui->comboBox_tools_action->currentText();
    E_WeldAction action = eNone_Action;
    if(action_str == "eInitAction"		)action = eInitAction;
    else if(action_str == "eGrind_MovorOff"	)action = eGrind_MovorOff;
    else if(action_str == "eGrind_OnorDown"	)action = eGrind_OnorDown;
    else if(action_str == "eGrind_Up"		)action = eGrind_Up;
    else if(action_str == "eWeld_MovorDwon"	)action = eWeld_MovorDwon;
    else if(action_str == "eWeld_Fix"		)action = eWeld_Fix;
    else if(action_str == "eWeld_Up"		)action = eWeld_Up;
    else if(action_str == "eWeld_On"		)action = eWeld_On;
    else if(action_str == "eWeld_Down"		)action = eWeld_Down;
    else                                     action = eNone_Action;

    if(ui->check_connect->isChecked())
    {
        m_Com->SetGunConnect(index);

    }else
    {
        m_Com->SetGunConnect(0);
    }
    m_Com->SetToolsAction(index,action);

}

void MainWindow::on_comboBox_magents_currentIndexChanged() {

    int index = ui->comboBox_magents_->currentIndex();
    QString action_str = ui->comboBox_magents_action->currentText();
    E_MagentAction action = eNONE_Magent;
    if(action_str == "eNONE_Magent"	)   action = eNONE_Magent;
    else if(action_str == "eMag_On"	)   action = eMag_On;
    else if(action_str == "eMag_Off")  action = eMag_Off;
    else if(action_str == "eMag_Up"	)   action = eMag_Up;
    else if(action_str == "eMag_Down")  action = eMag_Down;
    else                                action = eNONE_Magent;

    m_Com->SetMagentAction(index,action);

}

void MainWindow::btn_moveFwd_shaft_released() {

    m_Robot->setLinkHalt();
    m_Com->LinkHalt(1);

}

void MainWindow::btn_moveBwd_shaft_pressed() {


    QPushButton *pressedButton = qobject_cast<QPushButton*>(sender());
    if (!pressedButton){
        return;
    }
    setButtonIndex();

    // 获取按钮在数组中的索引
    int index = -1;
    for (int i = 0; i < jointNum; i++) {
        if (pressedButton == findChild<QPushButton*>("btn_moveBwd_shaft" + QString::number(i))) {
            index = i;
            break;
        }
    }

    if(index>=0 && index< jointNum)
    {
//        m_Robot->setJointMoveVel(index,-0.1* axisVelLimit[index][1]);
        m_Robot->setJointMoveVel(index,-0.5* LINK_0_JOINT_MAX_VEL[index]);
        this->logger->info("************设置轴{}速度为{}****************", index, -0.5* LINK_0_JOINT_MAX_VEL[index]);
    }

}

void MainWindow::btn_moveBwd_shaft_released() {
    m_Robot->setLinkHalt();
    m_Com->LinkHalt(1);
}

void MainWindow::on_moveRel_shaft_clicked() {

    QPushButton *pressedButton = qobject_cast<QPushButton*>(sender());
    if (!pressedButton) {
        return;
    }
    setButtonIndex();

    // 获取按钮在数组中的索引
    int index = -1;
    for (int i = 0; i < jointNum; i++) {
        if (pressedButton == findChild<QPushButton*>("btn_moveRel_shaft" + QString::number(i))) {
            index = i;
            break;
        }
    }

    //axis状态更新
    QVector<st_ReadAxis> stJointstatus = m_Com->getLinkJointStatus(0); //link轴组数据
    QVector<st_ReadAxis> stJointPitch = m_Com->getLinkJointStatus(1);//腰部俯仰数据
    stJointstatus.append(stJointPitch.back());
    double vel =  0.2 * LINK_0_JOINT_MAX_VEL[index];

    if(index>=0 && index< jointNum)
    {
        double delta = findChild<QLineEdit*>("lineEdit_setPos_shaft" + QString::number(index))->text().toDouble();
        if(delta < 0) vel = -vel;
        logger->info(("[ on_moveRel_shaft_clicked ]  index : " + std::to_string(index) + " delta:"+std::to_string(delta) + " vel:"+ std::to_string(vel)).c_str());
        m_Robot->setJointMoveAbs(index , stJointstatus[index].Position + delta  , vel);
    }
    findChild<QLineEdit*>("lineEdit_setPos_shaft" + QString::number(index))->setText("0");
}

void MainWindow::btn_moveFwd_end_pressed() {

    double vel[6] = {0};
    QPushButton *pressedButton = qobject_cast<QPushButton*>(sender());
    if (!pressedButton) {
        return;
    }

    int index = -1;
    for (int i = 0; i < 6; i++) {
        if (pressedButton == findChild<QPushButton*>("btn_moveFwd_end" + QString::number(i))) {
            index = i;
            break;
        }
    }

    if(index >=0 && index<=5){
        if(index <= 2){
            vel[index] = GP::velLine;
        }else{
            vel[index] = GP::velRotate;
        }
        logger->info(("[ btn_moveFwd_end_pressed ]  index : " + std::to_string(index) + " vel:"+ std::to_string(vel[index])).c_str());
        m_Robot->setLinkMoveVel(vel);
    }else{
        qDebug()<<"【ERROR】 void MainWindow::on_btn_setLinkMoveVel_pressed() ";
    }
}

void MainWindow::btn_moveFwd_end_released() {
    m_Robot->setLinkHalt();
}

void MainWindow::btn_moveBwd_end_pressed() {
    double vel[6] = {0};
    QPushButton *pressedButton = qobject_cast<QPushButton*>(sender());
    if (!pressedButton) {
        return;
    }

    int index = -1;
    for (int i = 0; i < 6; i++) {
        if (pressedButton == findChild<QPushButton*>("btn_moveBwd_end" + QString::number(i))){
            index = i;
            break;
        }
    }

    if(index >=0 && index<=5){
        if(index<=2){
            vel[index] = -GP::velLine;
        }else{
            vel[index] = -GP::velRotate;
        }
        m_Robot->setLinkMoveVel(vel);
        logger->info(("[ btn_moveBwd_end_pressed ]  index : " + std::to_string(index) + " vel:"+ std::to_string(vel[index])).c_str());

    }else{
        qDebug()<<"【ERROR】 void MainWindow::on_btn_setLinkMoveVel_pressed() ";
    }

}

void MainWindow::btn_moveBwd_end_released() {
    m_Robot->setLinkHalt();
}

void MainWindow::on_moveRel_end_clicked() {
    QPushButton *pressedButton = qobject_cast<QPushButton*>(sender());
    if (!pressedButton) {
        return;
    }

    // 获取按钮在数组中的索引
    int index = -1;
    for (int i = 0; i < jointNum; i++) {
        if (pressedButton == findChild<QPushButton*>("btn_moveRel_end" + QString::number(i))) {
            index = i;
            break;
        }
    }

    double pos[6] = {0};
    for(int i=0;i<3;i++)
    {
        pos[i] = findChild<QLineEdit*>("lineEdit_setPos_end" + QString::number(i))->text().toDouble() + findChild<QLabel*>("label_pos_end" + QString::number(i))->text().toDouble();
        int j = i+3;
        pos[j] = (findChild<QLineEdit*>("lineEdit_setPos_end" + QString::number(j))->text().toDouble() + findChild<QLabel*>("label_pos_end" + QString::number(j))->text().toDouble())/57.3;
    }

    for(int i=0;i<6;i++)
    {
        logger->info(("[ on_moveRel_end_clicked ]  index : " + std::to_string(i) + " pos:"+ std::to_string(pos[i])).c_str());
    }

    m_Robot->setLinkMoveAbs(pos,GP::End_Vel_Limit.data());
    logger->info("[ on_moveRel_end_clicked ]  m_Robot->setLinkMoveAbs(pos,GP::End_Vel_Limit);");

    for(int i=0;i<6;i++)
    {
        findChild<QLineEdit*>("lineEdit_setPos_end" + QString::number(i))->setText("0");
    }
}

void MainWindow::on_btn_SetTools_clicked()
{
    int index = ui->comboBox_tools_->currentIndex()+1;
    QString action_str = ui->comboBox_tools_action->currentText();
    E_WeldAction action = eNone_Action;
    if(action_str == "eInitAction"		)    action = eInitAction;
    else if(action_str == "eGrind_MovorOff"	)action = eGrind_MovorOff;
    else if(action_str == "eGrind_OnorDown"	)action = eGrind_OnorDown;
    else if(action_str == "eGrind_Up"		)action = eGrind_Up;
    else if(action_str == "eWeld_MovorDwon"	)action = eWeld_MovorDwon;
    else if(action_str == "eWeld_Fix"		)action = eWeld_Fix;
    else if(action_str == "eWeld_Up"		)action = eWeld_Up;
    else if(action_str == "eWeld_On"		)action = eWeld_On;
    else if(action_str == "eWeld_Down"		)action = eWeld_Down;
    else                                     action = eNone_Action;


    if(ui->check_connect->isChecked())
    {
        m_Com->SetGunConnect(index);

    }else
    {
        m_Com->SetGunConnect(0);
    }
    m_Com->SetToolsAction(index,action);
}

void MainWindow::on_btn_SetMagent_clicked()
{
    int index = ui->comboBox_magents_->currentIndex();
    QString action_str = ui->comboBox_magents_action->currentText();
    E_MagentAction action = eNONE_Magent;
    if(action_str == "eNONE_Magent"	)   action = eNONE_Magent;
    else if(action_str == "eMag_On"	)   action = eMag_On;
    else if(action_str == "eMag_Off")  action = eMag_Off;
    else if(action_str == "eMag_Up"	)   action = eMag_Up;
    else if(action_str == "eMag_Down")  action = eMag_Down;
    else                                action = eNONE_Magent;

    m_Com->SetMagentAction(index,action);
}

void MainWindow::updateActionSta() {
    //半自动作业按钮状态更新
//    static int old_action_index = -100;
//    int action_index = m_Task->ActionIndex.loadRelaxed();
//    const std::map<int,QString> AUTOWORK_NAME = {
//            {10,"btn_location_"}, {12,"btn_lift_"}, {15,"btn_leveling_"}, {16,"btn_lift_2"},
//            {20,"btn_sideline_"}, {35,"btn_sideline_"}, {28,"btn_sideline_"}, {30,"btn_magnet_open_"},
//            {40,"btn_auto_knock_"}, {32,"btn_magnet_close_"}, {42,"btn_magnet_pause_"}, {44,"btn_knock_suspend_"},
//    };
//
////    const std::map<int,std::string> AUTOWORK_NAME = {
////            {10,"准备位置"},{12,"举升"},{15,"调平"},{16,"举升去对边"},
////            {20,"获取边线"},{35,"调整偏差"},{28,"检测调整结果"},{30,"开启磁铁"},
////            {40,"自动碰钉"},{32,"关闭磁铁"},{42,"碰钉暂停"},{44,"碰钉中止"}
////    };
//
//    //当action_index状态改变时才刷新按钮
////    logger->info("action_index {}",action_index);
//    if(action_index == 0){
//        for(auto it=AUTOWORK_NAME.begin();it!=AUTOWORK_NAME.end();++it){
//            findChild<QPushButton*>(it->second)->setStyleSheet("background-color: rgb(170, 170, 255)");
//        }
//    }
//
////    ui->btn_camera_close->setText(QString::number(action_index));//临时显示action_index
//
//    if(old_action_index != action_index){
//        old_action_index = action_index;
//        for(auto it=AUTOWORK_NAME.begin();it!=AUTOWORK_NAME.end();++it){
//            if(it->first == action_index){
//                findChild<QPushButton*>(it->second)->setStyleSheet("background-color: green; color: black;");
//            }else{
//                findChild<QPushButton*>(it->second)->setStyleSheet("background-color: rgb(170, 170, 255)");
//            }
//        }
//    }
}

void MainWindow::updateConnectSta() {
    std::map<QString,const bool> comStatus = {
            {"icon_connect_plc",m_Com->getCommState_Robot()},
            {"icon_connect_io",m_Com->getCommState_IOA()},
            {"icon_connect_io",m_Com->getCommState_IOB()},
    };

    for(const auto&item: comStatus){
        if(item.second == false){
            findChild<QLabel*>(item.first)->setStyleSheet("image: url(:/img/images/icon_redLight.png);");
        }else{
            findChild<QLabel*>(item.first)->setStyleSheet("image: url(:/img/images/icon_greenLight.png);");
        }
    }
}

void MainWindow::on_btn_line_detect_clicked() {
//    bool isEnable = ui->btn_camera_open->isEnabled();
//    if(isEnable){
//        this->logger->info("相机未开启");
//        return;
//    }
    unsigned pageIndex = ui->stackedWidget_view->currentIndex();
    if(pageIndex!=1){
        return ;
    }

    // 1. 设置直线检测位
    this->setLineStatus(true);

    std::map<std::string, LineDetectRes> res = m_VisionInterface->getLineRes();
    for(const auto &item : res){
        std::string prefix =  item.first;
        size_t index = prefix.find("_");
        int number = prefix[index+1]-'0';
        float dist = item.second.dist*m_VisionInterface->scales_line[prefix];
        // 更新直线检测结果
        findChild<QLabel*>("label_line_dist" + QString::number(number-1))->setText("Dist " + QString::number(number) + ":" + QString::number(dist));

        // 更新摄像头数据
        cv::Mat temp;
        if(!item.second.img_drawed.empty()){
            cv::resize(item.second.img_drawed, temp, cv::Size(400, 280));
            QImage img = QImage((uchar*)temp.data, temp.cols, temp.rows, QImage::Format_RGB888);
            findChild<QLabel*>("label_camera_dis" + QString::number(number - 1))->setPixmap(QPixmap::fromImage(img));
        }
    }
}

void MainWindow::on_btn_line_detect_debug_clicked() {
    ui->stackedWidget_view->setCurrentIndex(1);
}

void MainWindow::on_btn_camera_capture_clicked() {
    this->setLineStatus(false);
}

void MainWindow::on_btn_camera_save_clicked() {
    std::string saveRoot = "../cache/";
    this->m_VisionInterface->camera_controls->getImageAll();
    std::map<std::string, cv::Mat> cameraData = this->m_VisionInterface->camera_controls->getCameraImages();
    for(const auto &item : cameraData) {
        std::string timeStr = getCurrentTimestampString();
        std::string img_path = saveRoot + item.first+"_"+timeStr+".png";
        if(!item.second.empty()){
            cv::imwrite(img_path, item.second);
        }
    }

}

bool MainWindow::getLineStatus() {
    bool status;
    this->m_mutex.lock();
    status = MainWindow::lineStatus;
    this->m_mutex.unlock();
    return status;
}

void MainWindow::setLineStatus(bool lineStatus) {
    this->m_mutex.lock();
    MainWindow::lineStatus = lineStatus;
    this->m_mutex.unlock();
}

void MainWindow::setButtonIndex() {

    QPushButton *button = qobject_cast<QPushButton*>(sender());
    QString objectName;
    if (button) {
        objectName = button->objectName();
    }else{
        this->logger->error("Button is null!");
        return ;
    }
    //设置原子变量index
    if(this->m_btnIndex.find(objectName.toStdString()) != this->m_btnIndex.end()){
        m_Task->ButtonIndex.storeRelaxed(this->m_btnIndex[objectName.toStdString()]);
//        this->logger->info("The object name of the clicked button is:{}",objectName.toStdString());
    }else{
        this->logger->error("Button index not found in predefined index map!");
    }
}

void MainWindow::setActionIndex() {
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    QString objectName;
    if (button) {
        objectName = button->objectName();
    }else{
        this->logger->error("Button is null!");
        return ;
    }
    //设置原子变量index
    if(this->m_jobBtnIndex.find(objectName.toStdString()) != this->m_jobBtnIndex.end()){
        m_Task->ActionIndex.storeRelaxed(this->m_jobBtnIndex[objectName.toStdString()]);
        this->logger->info("The object name of the clicked button is:{}",objectName.toStdString());
    }else{
        this->logger->error("Button index not found in predefined index map!");
    }
}

void MainWindow::slots_on_btn_magnet_exit_clicked() {
    setButtonIndex();
    setActionIndex();
    this->logger->info("退出");

}

void MainWindow::slots_on_btn_add_nail_clicked() {
    setButtonIndex();
    setActionIndex();
    this->logger->info("放钉");

}

void MainWindow::on_btn_wheel_forward_pressed() {

    // 底盘前进
    m_Robot->setJointMoveVel(GP::WHEEL_LEFT_INDEX, m_wheelVel);
    m_Robot->setJointMoveVel(GP::WHEEL_RIGHT_INDEX, m_wheelVel);
    this->logger->info("******前进*******");

}

void MainWindow::on_btn_wheel_backward_released() {
    // 停止前进
    m_Com->JointHalt(GP::WHEEL_LEFT_INDEX);
    m_Com->JointHalt(GP::WHEEL_RIGHT_INDEX);
}

void MainWindow::on_btn_wheel_left_pressed() {
    m_Robot->setJointMoveVel(GP::WHEEL_LEFT_INDEX, -10);
    m_Robot->setJointMoveVel(GP::WHEEL_RIGHT_INDEX, 10);
    this->logger->info("******左转*******");
}

void MainWindow::on_btn_wheel_right_released() {
    m_Com->JointHalt(GP::WHEEL_LEFT_INDEX);
    m_Com->JointHalt(GP::WHEEL_RIGHT_INDEX);
}

void MainWindow::on_btn_wheel_forward_released() {
    m_Com->JointHalt(GP::WHEEL_LEFT_INDEX);
    m_Com->JointHalt(GP::WHEEL_RIGHT_INDEX);
}

void MainWindow::on_btn_wheel_backward_pressed() {
    m_Robot->setJointMoveVel(GP::WHEEL_LEFT_INDEX, -m_wheelVel);
    m_Robot->setJointMoveVel(GP::WHEEL_RIGHT_INDEX, -m_wheelVel);
    this->logger->info("******后退*******");
}

void MainWindow::on_btn_wheel_left_released() {
    m_Com->JointHalt(GP::WHEEL_LEFT_INDEX);
    m_Com->JointHalt(GP::WHEEL_RIGHT_INDEX);
}

void MainWindow::on_btn_wheel_right_pressed() {
    m_Robot->setJointMoveVel(GP::WHEEL_LEFT_INDEX, 10);
    m_Robot->setJointMoveVel(GP::WHEEL_RIGHT_INDEX, -10);
    this->logger->info("******右转*******");
}

void MainWindow::on_btn_steering_left_pressed() {
    double angle = findChild<QLabel*>("label_steering_pos")->text().toDouble() +5;
    angle = angle > 90 ? 90 : angle;
    m_Robot->setJointMoveAbs(GP::STEER_LEFT_INDEX, angle ,5);
    m_Robot->setJointMoveAbs(GP::STEER_RIGHT_INDEX, angle ,5);



}

void MainWindow::on_btn_steering_left_released() {


}

void MainWindow::on_btn_steering_right_pressed() {
    double angle = findChild<QLabel*>("label_steering_pos")->text().toDouble() -5;
    angle = angle <- 90 ? -90 : angle;
    m_Robot->setJointMoveAbs(GP::STEER_LEFT_INDEX, angle ,5);
    m_Robot->setJointMoveAbs(GP::STEER_RIGHT_INDEX, angle ,5);
}

void MainWindow::on_btn_steering_right_released() {

}

void MainWindow::on_btn_add_nail_pressed() {

   double vel_Home[10] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
   for(int i = 0;i<10;i++){
       vel_Home[i] = LINK_0_JOINT_MAX_VEL[i]*0.1;
   }
   m_Robot->setJointGroupMoveAbs(GP::Position_Map[{GP::Working_Scenario, GP::PositionType::Prepare}].value.data(),vel_Home);

}

void MainWindow::on_btn_add_nail_released() {
    m_Robot->setLinkHalt();
}

void MainWindow::on_btn_preparation_pos_pressed() {
    double vel_pre[10] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    for(int i = 0;i<10;i++){
        vel_pre[i] = LINK_0_JOINT_MAX_VEL[i]*0.1;
    }
    m_Robot->setJointGroupMoveAbs(GP::Position_Map[{GP::Working_Scenario, GP::PositionType::Lift}].value.data(),vel_pre);
}

void MainWindow::on_btn_preparation_pos_released() {
    m_Robot->setLinkHalt();
}


void MainWindow::slots_btn_load_configuration_clicked()
{
    if (m_config_ptr->ReloadConfiguration())
    {
        ui->btn_load_configuration->setStyleSheet("background-color: rgb(0, 255, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
    }
    else
    {
        ui->btn_load_configuration->setStyleSheet("background-color: rgb(255, 0, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
    }
}

void MainWindow::slots_btn_save_home_position_clicked()
{
    bool res{ true };
    auto status = m_Robot->getJointGroupSta();
    std::vector<double> data;
    for (auto& i : status)
    {
        if (i.eState != eAxis_STANDSTILL)
        {
            res = false;
            break;
        }
        data.push_back(i.Position);
    }
    if (!res)
    {
        ui->btn_save_home_position->setStyleSheet("background-color: rgb(255, 0, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
        return;
    }
    GP::Position_Map[{GP::Working_Scenario, GP::PositionType::Prepare}].value = data;
    res = m_config_ptr->UpdateValue("position_map", GP::Position_Map);

    if (res)
    {
        ui->btn_save_home_position->setStyleSheet("background-color: rgb(0, 255, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
    }
    else
    {
        ui->btn_save_home_position->setStyleSheet("background-color: rgb(255, 0, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
    }
}

void MainWindow::slots_btn_save_prepare_position_clicked()
{
    bool res{ true };
    auto status = m_Robot->getJointGroupSta();
    std::vector<double> data;
    for (auto& i : status)
    {
        if (i.eState != eAxis_STANDSTILL)
        {
            res = false;
            break;
        }
        data.push_back(i.Position);
    }
    if (!res)
    {
        ui->btn_save_prepare_position->setStyleSheet("background-color: rgb(255, 0, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
        return;
    }
    GP::Position_Map[{GP::Working_Scenario, GP::PositionType::Lift}].value = data;
    res = m_config_ptr->UpdateValue("position_map", GP::Position_Map);

    if (res)
    {
        ui->btn_save_prepare_position->setStyleSheet("background-color: rgb(0, 255, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
    }
    else
    {
        ui->btn_save_prepare_position->setStyleSheet("background-color: rgb(255, 0, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
    }
}

void MainWindow::slots_btn_save_quit_position_clicked()
{
    bool res{ true };
    auto status = m_Robot->getJointGroupSta();
    std::vector<double> data;
    for (auto& i : status)
    {
        if (i.eState != eAxis_STANDSTILL)
        {
            res = false;
            break;
        }
        data.push_back(i.Position);
    }
    if (!res)
    {
        ui->btn_save_quit_position->setStyleSheet("background-color: rgb(255, 0, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
        return;
    }
    GP::Position_Map[{GP::Working_Scenario, GP::PositionType::Quit}].value = data;
    res = m_config_ptr->UpdateValue("position_map", GP::Position_Map);

    if (res)
    {
        ui->btn_save_quit_position->setStyleSheet("background-color: rgb(0, 255, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
    }
    else
    {
        ui->btn_save_quit_position->setStyleSheet("background-color: rgb(255, 0, 0);"
            "border: 2px solid blue;"
            "border-radius: 10px;"
        );
    }
}
