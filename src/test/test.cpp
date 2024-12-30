#include "test.hpp"

int line_detect_demo(int argc, char* argv[])
{
    std::string path = "E:\\projects\\ZBRobot\\2024-12-23\\robot\\cache\\LineCam_4_2024-12-24-19-04-41.png";
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
    std::vector<double> laser_distance{90, 90, 90, 90};

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

        auto result = std::find_if(BOARDING_MOTION_QUE.begin(), BOARDING_MOTION_QUE.end(), [laser_distance](double val){return laser_distance.at(0) == val;});
        if (result != BOARDING_MOTION_QUE.end())
        {
            // 利用计数器，休眠1s
            static int cnt{0};
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

        std::for_each(laser_distance.begin(), laser_distance.end(), [](double& val){val -= 1;});

    }

    std::for_each(laser_distance.begin(), laser_distance.end(), [](double& val){SPDLOG_INFO(val);});

    return 0;
}