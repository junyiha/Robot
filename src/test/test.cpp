#include "test.hpp"

int line_detect_demo(int argc, char* argv[])
{
    std::string path = "C:/Users/anony/Documents/GitHub/cpp-win/data/LineCam_6_2024-12-23-15-59-55.png";
    cv::Mat img = cv::imread(path);
    LineDetector line_tool;
    auto start = std::chrono::high_resolution_clock::now();
    LineSpaceResult res = line_tool.getLinesDistance(img);
    auto end = std::chrono::high_resolution_clock::now();

    // 计算并输出运行时间
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "程序运行时间: " << elapsed_seconds.count() << " 秒" << std::endl;
    if (res.status)
    {
        cv::imshow("img", res.img_drawed);
        cv::waitKey(0);
    }
    else
    {
        cv::imshow("img", res.img_drawed);
        cv::waitKey(0);
        std::cout << res.error_info << std::endl;
    }

    return 0;
}

int laserDemo(int argc, char* argv[])
{
    const char* port = "COM2";
    LaserDistanceBojke laserTool;
    bool ret = laserTool.open(port);

    while (true)
    {
        LaserMeasureData res = laserTool.getLaserMeasureData();
        std::cout << "*************************start*****" << std::endl;
        for (int i = 0; i < 4; i++)
        {
            std::cout << "is vaild:" << res.m_bLaserdistance[i] << "  value:" << res.m_Laserdistance[i] << std::endl;
        }
        std::cout << "*************************end*****" << std::endl;
    }

    QThread::sleep(3000 * 10);
    return 0;
}

int TestCom(int argc, char* argv[])
{
    auto  m_Com = ComInterface::getInstance();
    m_Com->start();
    while (true)
    {
        std::string cmd;
        std::cerr << "input command: \n";
        std::cin >> cmd;
        if (cmd == "open")
        {
            m_Com->SetLight(1, true);
        }
        else if (cmd == "close")
        {
            m_Com->SetLight(1, false);
        }
        else
        {
            std::cerr << "invalid command: " << cmd << "\n";
        }
    }

    return 0;
}

int TestTask(int argc, char* argv[])
{
    struct Data_t
    {
        std::vector<double> m_LineDistance;
        std::vector<bool> m_bLineDistance;
    };
    Data_t m_stMeasuredata;
    m_stMeasuredata.m_LineDistance = { 14.118399342577506,15.141133164344948,14.899653504249896,15.315110857435997,16.001300774736606,12.97208101039237 };
    m_stMeasuredata.m_bLineDistance = std::vector<bool>(6, true);
    double line_dis_1 = ((m_stMeasuredata.m_LineDistance[0] - 15) * m_stMeasuredata.m_bLineDistance[0] - (m_stMeasuredata.m_LineDistance[1] - 15) * m_stMeasuredata.m_bLineDistance[1]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[0]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[1]));
    double line_dis_2 = ((m_stMeasuredata.m_LineDistance[2] - 15) * m_stMeasuredata.m_bLineDistance[2] - (m_stMeasuredata.m_LineDistance[3] - 15) * m_stMeasuredata.m_bLineDistance[3]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[2]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[3]));
    double line_dis_3 = ((m_stMeasuredata.m_LineDistance[4] - 15) * m_stMeasuredata.m_bLineDistance[4] - (m_stMeasuredata.m_LineDistance[5] - 15) * m_stMeasuredata.m_bLineDistance[5]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[4]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[5]));

    std::cerr << "line_dis_1: " << line_dis_1 << "\n"
        << "line_dis_2: " << line_dis_2 << "\n"
        << "line_dis_3: " << line_dis_3 << "\n";

    return 0;
}

int TestInitLog(int argc, char* argv[])
{
    //创建控制台日志记录器
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);

    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e][thread %t][%@,%!] [%l] : %v");

    // 创建文件日志记录器: 滚动记录，最大文件5M，文件数量100个
    std::string log_path = ROOT_PATH;
    log_path += "logs/rotating.txt";
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path, 1048576 * 5, 100);

    //同步记录器，
    std::vector<spdlog::sink_ptr> sinks{ console_sink, rotating_sink };
    auto logger = std::make_shared<spdlog::logger>("logger", sinks.begin(), sinks.end());

    spdlog::register_logger(logger); //注册为全局日志，通过log_write访问;
    spdlog::flush_every(std::chrono::seconds(3)); //每3s刷新一次
    //根据需要调整记录级别：调试debug，发布info
    spdlog::set_level(spdlog::level::debug);

    return 0;
}

int TestConfigManager(int argc, char* argv[])
{
    TestInitLog(argc, argv);

    auto config_manager = std::make_unique<Config::ConfigManager>();

    for (auto& it : GP::Position_Map)
    {
        std::cerr << "work brief: " << it.second.brief << "\n";
        std::for_each(it.second.value.begin(), it.second.value.end(), [](double val) {std::cerr << val << ", "; });
        std::cerr << "\n";
    }
    return 0;
}

int TestTcpClient(int argc, char* argv[])
{
    asio::io_context io_context;
    asio::ip::tcp::resolver resolver(io_context);
    auto address = asio::ip::make_address("127.0.0.1");
    asio::ip::tcp::endpoint endpoint(address, 9990);
    net::TcpClient client(io_context, endpoint);
    std::thread t([&io_context]() { while (true) { io_context.run(); }});

    std::vector<char> buf(1024);
    while (true)
    {
        if (!client.IsConnect())
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        std::cerr << "input message: \n";
        std::cin.getline(buf.data(), buf.size());
        client.Write(buf);
        std::cerr << "receive data: " << client.Read().data() << "\n";
    }

    return 0;
}

int CheckParallelState(std::vector<double> laserDistance, int max_deviation, int min_deviation, int lift_distance)
{
    // 检查输入数据
    if (laserDistance.size() < 4)
    {
        SPDLOG_ERROR("激光数据数量输入有误");
        return -1;
    }
    for (int i = 0; i < 4; ++i)
    {
        if (laserDistance[i] > 450 || laserDistance[i] < -3)
        {
            SPDLOG_ERROR("激光数据有误,或壁面距离太远, 激光编号: {}, 数值: {}", i, laserDistance.at(i));
            return -1;
        }
    }

    // 计算激光距离最大偏差
    auto max_res = std::max_element(laserDistance.begin(), laserDistance.end());
    auto min_res = std::min_element(laserDistance.begin(), laserDistance.end());
    auto laser_average = std::accumulate(laserDistance.begin(), laserDistance.end(), 0) / laserDistance.size();

    if (*max_res - *min_res > max_deviation && *min_res / *max_res < 0.5)
    {
        SPDLOG_ERROR("激光距离最大偏差大于{}mm", max_deviation);
        return -1;
    }
    // 判断是否完成调平
    if (std::fabs(*max_res - laser_average) < min_deviation &&
        std::fabs(*min_res - laser_average) < min_deviation &&
        *min_res < lift_distance) // 最大偏差小于阈值
        return 1;
    else
        return 0;
}

int TestFitBoard(int argc, char* argv[])
{
    const std::vector<double> BOARDING_MOTION_QUE = { 60, 40, 30, 20, 15, 10, 5, 0 }; // 贴合运动序列
    std::vector<double> laser_distance{ 90, 90, 90, 90 };

    auto config_ptr = std::make_unique<Config::ConfigManager>();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto motion_index = BOARDING_MOTION_QUE.size();
        auto min_distance = std::min_element(laser_distance.begin(), laser_distance.end());
        for (int i = 0; i < BOARDING_MOTION_QUE.size(); ++i)
        {
            if (BOARDING_MOTION_QUE[i] + 2 < *min_distance)
            {
                motion_index = i;
                SPDLOG_INFO("贴合调整目标距离为：{}", BOARDING_MOTION_QUE[i]);
                break;
            }
        }

        int res = CheckParallelState(laser_distance, GP::Max_Deviation_In_FitBoard, GP::Min_Deviation_In_FitBoard, GP::Lift_Distance_In_FitBoard);
        if (res == 1)
        {
            SPDLOG_INFO("贴合--检测 ==> 贴合--贴合完成");
            break;
        }

        auto result = std::find_if(BOARDING_MOTION_QUE.begin(), BOARDING_MOTION_QUE.end(), [laser_distance](double val) {return laser_distance.at(0) == val; });
        if (result != BOARDING_MOTION_QUE.end())
        {
            // 利用计数器，休眠1s
            static int cnt{ 0 };
            SPDLOG_INFO("Count: {}", cnt);
            if (cnt > 20)
            {
                SPDLOG_INFO("开始对边运动...");
                cnt = 0;
                goto __NEXT;
            }
            else
            {
                cnt++;
                continue;
            }
        }
    __NEXT:

        std::for_each(laser_distance.begin(), laser_distance.end(), [](double& val) {val -= 1; });

    }

    std::for_each(laser_distance.begin(), laser_distance.end(), [](double& val) {SPDLOG_INFO(val); });

    return 0;
}

int TestRobot(int argc, char* argv[])
{
    auto robot_ptr = std::make_unique<CRobot>(new ComInterface());
    robot_ptr->start();

    std::this_thread::sleep_for(std::chrono::seconds(10));

    auto current_link_status = robot_ptr->getLinkSta();

    std::vector<double> cur_pos(current_link_status.stLinkActKin.LinkPos, current_link_status.stLinkActKin.LinkPos + 6);

    cur_pos.at(2) += 50;

    std::vector<double> max_vel{ 10, 10, 10, 0.5, 0.5, 0.5 };

    while (true)
    {
        robot_ptr->setLinkMoveAbs(cur_pos.data(), max_vel.data());

        if (robot_ptr->isEndReached(cur_pos))
        {
            SPDLOG_INFO("到达目标位置");
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}

int TestQtSQL(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    QStringList drivers = QSqlDatabase::drivers();
    qDebug() << "Supported database drivers:";
    for (const QString& driver : drivers)
    {
        qDebug() << driver;
    }

    QString db_path = ROOT_PATH;
    db_path += "configurations/robot.db";

#ifdef C_TEST


    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");


    db.setDatabaseName(db_path);

    if (!db.open())
    {
        std::cerr << "Failed to open database: " << db.lastError().text().toStdString() << "\n";
        return -1;
    }

    QString createTableSQL = R"(
        CREATE TABLE IF NOT EXISTS user (id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL, age INTEGER NOT NULL)
    )";
    QSqlQuery query;

    if (!query.exec(createTableSQL))
    {
        std::cerr << "Failed to create table: " << db.lastError().text().toStdString() << "\n";
        return -1;
    }

    QString insertDataSQL = "INSERT INTO user (name, age) VALUES(:name, :age)";
    query.prepare(insertDataSQL);
    query.bindValue(":name", "Alice");
    query.bindValue(":age", 25);
    if (!query.exec())
    {
        std::cerr << "Failed to create table: " << db.lastError().text().toStdString() << "\n";
        return -1;
    }

    QString selectDataSQL = "SELECT id, name, age FROM user";
    if (!query.exec(selectDataSQL))
    {
        std::cerr << "Failed to create table: " << db.lastError().text().toStdString() << "\n";
        return -1;
    }
    while (query.next())
    {
        int id = query.value("id").toInt();
        QString name = query.value("name").toString();
        int age = query.value("age").toInt();
        std::cerr << "ID: " << id << ", Name: " << name.toStdString() << ", Age: " << age << "\n";
    }

    db.close();
#endif

#ifdef TEST_SQL_INIT
    bool res{ false };
    UTILS::Sql sql(db_path);

    QString createTableSQL = R"(
        CREATE TABLE IF NOT EXISTS user (id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL, age INTEGER NOT NULL)
    )";

    res = sql.CreateTable(createTableSQL);
    if (!res)
    {
        return -1;
    }

    QSqlQuery query;
    QString insertDataSQL = "INSERT INTO user (name, age) VALUES(:name, :age)";
    query.prepare(insertDataSQL);
    query.bindValue(":name", "Ack");
    query.bindValue(":age", 16);
    res = sql.InsertData(query);
    if (!res)
    {
        return -1;
    }
#endif
    Utils::Sql sql(db_path);

    QString cmd = "SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_&'";
    QSqlQuery query;
    if (!query.exec(cmd))
    {
        SPDLOG_ERROR("Failed to execute query: {}", query.lastError().text().toStdString());
        return -1;
    }

    while (query.next())
    {
        QString table_name = query.value(0).toString();
        SPDLOG_INFO("table name: {}", table_name.toStdString());
    }

    cmd = "SELECT * FROM insulation_panel";
    if (!query.exec(cmd))
    {
        SPDLOG_ERROR("Failed to execute query: {}", query.lastError().text().toStdString());
        return -1;
    }
    struct Data_t
    {
        std::string id;
        std::string line_distance;
        std::string point_laser;
        std::string profiler_laser;
        std::string time;
    };

    std::vector<Data_t> data_vec;
    while (query.next())
    {
        Data_t data;
        data.id = query.value(0).toString().toStdString();
        data.line_distance = query.value(1).toString().toStdString();
        data.point_laser = query.value(2).toString().toStdString();
        data.profiler_laser = query.value(3).toString().toStdString();
        data.time = query.value(4).toString().toStdString();
        data_vec.push_back(data);
    }

    for (auto& data : data_vec)
    {
        SPDLOG_INFO("\nid: {}\nline_distance: {}\npoint_laser: {}\nprofiler_laser: {}\ntime: {}\n",
                    data.id, data.line_distance, data.point_laser, data.profiler_laser, data.time);
        QJsonDocument json_doc;
        QJsonParseError json_parse_error;

        json_doc = QJsonDocument::fromJson(QByteArray(data.profiler_laser.c_str()), &json_parse_error);
        if (json_parse_error.error != QJsonParseError::NoError)
        {
            SPDLOG_ERROR("JSON Parse Error: {}", json_parse_error.errorString().toStdString());
            return -1;
        }

        SPDLOG_INFO("format json string: {}", json_doc.toJson(QJsonDocument::Indented).toStdString());
    }



    return 0;
}

int TestQtJSON(int argc, char* argv[])
{
    QCoreApplication a(argc, argv);

    QJsonObject json_obj;
    json_obj["name"] = "John Doe";
    json_obj["age"] = 30;
    json_obj["isStudent"] = false;

    QJsonDocument doc(json_obj);

    QString json_str = doc.toJson(QJsonDocument::Indented);

    SPDLOG_INFO("Json data: {}", json_str.toStdString());

    return 0;
}

int TestQtSerialPort(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    foreach(const QSerialPortInfo & info, QSerialPortInfo::availablePorts())
    {
        qDebug() << "Port:" << info.portName();
        qDebug() << "Description:" << info.description();
        qDebug() << "Manufacturer:" << info.manufacturer();
    }

    QString port;
    // QTextStream in(stdin);
    // QTextStream out(stdout);

    // out << QString::fromLocal8Bit("输入选择的端口: \n") << flush;
    // port = in.readLine();
    // out << port << flush;
    port = "COM1";

#ifdef C_TEST
    QSerialPort serial;
    serial.setPortName(port);
    serial.setBaudRate(QSerialPort::Baud115200);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    bool res = serial.open(QIODevice::ReadWrite);
    if (!res)
    {
        qDebug() << "Failed to open port: " << serial.portName() << ", error: " << serial.errorString();
        return -1;
    }

    qDebug() << "Success to open port: " << serial.portName() << "\n";

    QByteArray buf(7, 0);

    buf[0] = 0x07;
    buf[1] = 0x00;
    buf[2] = 0x0C;
    buf[3] = 0x00;
    buf[4] = 0x0A;
    buf[5] = 0x01;
    buf[6] = 0X1E;

    while (true)
    {
        qint64 bytesWritten = serial.write(buf);
        if (!serial.waitForBytesWritten(1000))  // 默认为异步，此处目的：改为同步，等待写入完毕，再读数据
        {
            qDebug() << "write timeout...";
            break;
        }
        QThread::msleep(30);
        QByteArray recv_data = serial.read(32);
        qDebug() << "send data: " << buf.size() << "\n"
            << "receive data: " << recv_data.size() << "\n";
    }
#else

    Utils::Serial serial;

    bool res = serial.Open(port);
    if (!res)
    {
        SPDLOG_ERROR("Failed to open port: {}", port.toStdString());
        return -1;
    }

    serial.Loop();
#endif

    qDebug() << "exit...";

    return app.exec();
}

int TestComSerialCom(int argc, char* argv[])
{
    CSerialCom serial;
    std::string port;
    // std::cerr << "input port: \n";
    // std::cin >> port;
    port = "COM1";


    // CManual manual;

    // manual.open(port.c_str());
    // manual.start();

    // system("pause");

    bool res = serial.open(port.c_str());
    if (!res)
    {
        SPDLOG_ERROR("Failed to open port: {}", port);
        return 1;
    }

    std::vector<uint8_t> buf(7, 0);

    buf[0] = 0x07;
    buf[1] = 0x00;
    buf[2] = 0x0C;
    buf[3] = 0x00;
    buf[4] = 0x0A;
    buf[5] = 0x01;
    buf[6] = 0X1E;

    // 0x07 0x00 0x0C 0x00 0x0A 0x01 0X1E

    while (true)
    {
        serial.write(buf.data(), buf.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        uint8_t buffData[100];
        int recvlen = serial.read(buffData);
        std::cerr << "send data: " << buf.size() << "\n"
            << "receive data: " << recvlen << "\n";
    }

    return 0;
}

int TestQTcpSocket(int argc, char* argv[])
{
    QTcpSocket socket;

    QString ip = "192.168.1.201";
    int port = 5999;

    socket.connectToHost(ip, port);

    if (socket.waitForConnected(5000))
    {
        qDebug() << "Successfully connected to the server!";
    }
    else
    {
        qDebug() << "Connection failed:" << socket.errorString();
    }

    auto state = socket.state();
    switch (state)
    {
    case QAbstractSocket::UnconnectedState:
        qDebug() << "Socket is not connected.";
        break;
    case QAbstractSocket::HostLookupState:
        qDebug() << "Looking up host.";
        break;
    case QAbstractSocket::ConnectingState:
        qDebug() << "Connecting to server.";
        break;
    case QAbstractSocket::ConnectedState:
        qDebug() << "Connected to server.";
        break;
    case QAbstractSocket::BoundState:
        qDebug() << "Socket is bound.";
        break;
    case QAbstractSocket::ClosingState:
        qDebug() << "Socket is closing.";
        break;
    case QAbstractSocket::ListeningState:
        qDebug() << "Socket is listening.";
        break;
    }

    QByteArray send_data(SEND_LEN, 0);
    send_data[0] = SEND_LEN;
    send_data[1] = 64;
    send_data[2] = 16;
    send_data[3] = 0;
    send_data[4] = 16;
    send_data[5] = 1;
    // send_data[14] = 112;
    int sum_index = SEND_LEN - 1;

    send_data[sum_index] = std::accumulate(send_data.begin(), send_data.begin() + sum_index, 0);
    qDebug() << "send_data[" << sum_index << "]: " << static_cast<int>(send_data[sum_index]) << "\n";


    qint64 send_len = socket.write(send_data);
    qDebug() << "send length: " << send_len << "\n";
    if (socket.waitForBytesWritten(3000))
    {
        if (socket.waitForReadyRead(3000))
        {
            QByteArray data = socket.readAll();
            qDebug() << "Data received: " << data << ", receive size: " << data.size() << "\n";
            for (int i = 0; i < data.size(); i++)
            {
                qDebug() << "data[" << i << "]: " << static_cast<int>(data[i]) << ", ";
            }
        }
        else
        {
            qDebug() << "No data received.";
        }
    }

    return 0;
}

int TestBoardingTool(int argc, char* argv[])
{
    BoardingTool boarding_tool;

    std::string ip = "192.168.1.201";
    int port = 5999;

    boarding_tool.m_cIOA.ConnectToServer(ip.c_str(), port);

    auto data = boarding_tool.getLaserDistance();
    for (auto& it : data)
    {
        SPDLOG_INFO("laser distance: {}", it);
    }

    system("pause");

    return 0;
}


/*Get Image buffer function, you can get the chunk infomation from frame infomation*/
void __stdcall ImageCallBackEx(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pFrameInfo)
    {
        // Print parse the timestamp information in the frame
        printf("ImageCallBack:FrameNum[%d], ExposureTime[%f], SecondCount[%d], CycleCount[%d], CycleOffset[%d]\n",
            pFrameInfo->nFrameNum, pFrameInfo->fExposureTime, pFrameInfo->nSecondCount, pFrameInfo->nCycleCount, pFrameInfo->nCycleOffset);

        MV_CHUNK_DATA_CONTENT* pUnparsedChunkContent = pFrameInfo->UnparsedChunkList.pUnparsedChunkContent;
        for (unsigned int i = 0; i < pFrameInfo->nUnparsedChunkNum; i++)
        {
            // Only the ID and length are printed, and the content needs to be parsed according to the instruction document
            printf("ChunkInfo[%d]: ChunkID[0x%x], ChunkLen[%d]\n", i, pUnparsedChunkContent->nChunkID, pUnparsedChunkContent->nChunkLen);
            pUnparsedChunkContent++;
        }
        printf("***********************************\n");
    }
}

// Wait for key press
void WaitForKeyPress(void)
{
    while (!_kbhit())
    {
        Sleep(10);
    }
    _getch();
}

bool IsColor(MvGvspPixelType enType)
{
    switch (enType)
    {
    case PixelType_Gvsp_BGR8_Packed:
    case PixelType_Gvsp_YUV422_Packed:
    case PixelType_Gvsp_YUV422_YUYV_Packed:
    case PixelType_Gvsp_BayerGR8:
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_BayerBG8:
    case PixelType_Gvsp_BayerGB10:
    case PixelType_Gvsp_BayerGB10_Packed:
    case PixelType_Gvsp_BayerBG10:
    case PixelType_Gvsp_BayerBG10_Packed:
    case PixelType_Gvsp_BayerRG10:
    case PixelType_Gvsp_BayerRG10_Packed:
    case PixelType_Gvsp_BayerGR10:
    case PixelType_Gvsp_BayerGR10_Packed:
    case PixelType_Gvsp_BayerGB12:
    case PixelType_Gvsp_BayerGB12_Packed:
    case PixelType_Gvsp_BayerBG12:
    case PixelType_Gvsp_BayerBG12_Packed:
    case PixelType_Gvsp_BayerRG12:
    case PixelType_Gvsp_BayerRG12_Packed:
    case PixelType_Gvsp_BayerGR12:
    case PixelType_Gvsp_BayerGR12_Packed:
        return true;
    default:
        return false;
    }
}

bool IsMono(MvGvspPixelType enType)
{
    switch (enType)
    {
    case PixelType_Gvsp_Mono10:
    case PixelType_Gvsp_Mono10_Packed:
    case PixelType_Gvsp_Mono12:
    case PixelType_Gvsp_Mono12_Packed:
        return true;
    default:
        return false;
    }
}

cv::Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData)
{
    cv::Mat srcImage;
    if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
    }
    else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        //        RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
    }
    else
    {
        printf("unsupported pixel format\n");
    }
    return srcImage;
}

int TestHKCamera(int argc, char* argv[])
{
    int res{ 0 };
    MV_CC_DEVICE_INFO_LIST device_list;

    res = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (res != MV_OK)
    {
        SPDLOG_ERROR("Failed to enumerate devices, error code: {}", res);
        return -1;
    }

    SPDLOG_INFO("find {} device", device_list.nDeviceNum);
    for (int i = 0; i < device_list.nDeviceNum; i++)
    {
        if (device_list.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            std::cerr << "ip: " << nIp1 << "." << nIp2 << "." << nIp3 << "." << nIp4 << "\n"
                << "user defined name: " << device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chUserDefinedName << "\n"
                << "serial number: " << device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber << "\n\n";
        }
    }

    int camera_index = 0;
    std::cerr << "input camera index: \n";
    std::cin >> camera_index;

    void* handle = nullptr;
    res = MV_CC_CreateHandle(&handle, device_list.pDeviceInfo[camera_index]);
    if (res != MV_OK)
    {
        SPDLOG_ERROR("Failed to create handle, error code: {}", res);
        return -1;
    }

    res = MV_CC_OpenDevice(handle);
    if (res != MV_OK)
    {
        SPDLOG_ERROR("Failed to open device, error code: {}", res);
        return -1;
    }

#ifdef CHUNK_DATA
    res = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
    if (res != MV_OK)
    {
        SPDLOG_ERROR("Failed to register image callback, error code: {}", res);
        return -1;
    }

    // Open Chunk Mode
    res = MV_CC_SetBoolValue(handle, "ChunkModeActive", true);
    if (MV_OK != res)
    {
        printf("Set Chunk Mode fail! res [0x%x]\n", res);
        return -1;
    }

    // Chunk Selector set as Exposure
    res = MV_CC_SetEnumValueByString(handle, "ChunkSelector", "Exposure");
    if (MV_OK != res)
    {
        printf("Set Exposure Chunk fail! res [0x%x]\n", res);
        return -1;
    }

    // Open Chunk Enable
    res = MV_CC_SetBoolValue(handle, "ChunkEnable", true);
    if (MV_OK != res)
    {
        printf("Set Chunk Enable fail! res [0x%x]\n", res);
        return -1;
    }

    // Chunk Selector set as Timestamp
    res = MV_CC_SetEnumValueByString(handle, "ChunkSelector", "Timestamp");
    if (MV_OK != res)
    {
        printf("Set Timestamp Chunk fail! res [0x%x]\n", res);
        return -1;
    }

    // Open Chunk Enable
    res = MV_CC_SetBoolValue(handle, "ChunkEnable", true);
    if (MV_OK != res)
    {
        printf("Set Chunk Enable fail! res [0x%x]\n", res);
        return -1;
    }

    // Chunk Selector set as Timestamp
    res = MV_CC_SetEnumValueByString(handle, "ChunkSelector", "Timestamp");
    if (MV_OK != res)
    {
        printf("Set Timestamp Chunk fail! res [0x%x]\n", res);
        return -1;
    }

    // Open Chunk Enable
    res = MV_CC_SetBoolValue(handle, "ChunkEnable", true);
    if (MV_OK != res)
    {
        printf("Set Chunk Enable fail! res [0x%x]\n", res);
        return -1;
    }

    // Set trigger mode as off
    res = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    if (MV_OK != res)
    {
        printf("Set Trigger Mode fail! res [0x%x]\n", res);
        return -1;
    }

    // Start grab image
    res = MV_CC_StartGrabbing(handle);
    if (MV_OK != res)
    {
        printf("Start Grabbing fail! res [0x%x]\n", res);
        return -1;
    }

    printf("Press a key to stop grabbing.\n");
    WaitForKeyPress();
#else

    unsigned char* pConvertData = NULL;
    unsigned int nConvertDataSize = 0;

    // Detection network optimal package size(It only works for the GigE camera)
    if (device_list.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0)
        {
            res = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
            if (res != MV_OK)
            {
                printf("Warning: Set Packet Size fail res [0x%x]!", res);
            }
        }
        else
        {
            printf("Warning: Get Packet Size fail res [0x%x]!", nPacketSize);
        }
    }

    res = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
    if (MV_OK != res)
    {
        printf("Set Trigger Mode fail! res [0x%x]\n", res);
        return -1;
    }

    // Start grab image
    res = MV_CC_StartGrabbing(handle);
    if (MV_OK != res)
    {
        printf("Start Grabbing fail! res [0x%x]\n", res);
        return -1;
    }

    MV_FRAME_OUT stImageInfo = { 0 };

    res = MV_CC_GetImageBuffer(handle, &stImageInfo, 1000);
    if (res != MV_OK)
    {
        printf("No data[0x%x]\n", res);
        return -1;
    }

    printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
        stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nFrameNum);

    cv::Mat src_img = Convert2Mat(&stImageInfo.stFrameInfo, stImageInfo.pBufAddr);
    cv::imshow("src_image", src_img);
    cv::waitKey(0);

    MV_CC_FreeImageBuffer(handle, &stImageInfo);

#endif

    // Stop grab image
    res = MV_CC_StopGrabbing(handle);
    if (MV_OK != res)
    {
        printf("Stop Grabbing fail! res [0x%x]\n", res);
        return -1;
    }

    // Close device
    res = MV_CC_CloseDevice(handle);
    if (MV_OK != res)
    {
        printf("ClosDevice fail! res [0x%x]\n", res);
        return -1;
    }

    // Destroy handle
    res = MV_CC_DestroyHandle(handle);
    if (MV_OK != res)
    {
        printf("Destroy Handle fail! res [0x%x]\n", res);
        return -1;
    }

    if (res != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("Press a key to exit.\n");
    WaitForKeyPress();

    return 0;
}