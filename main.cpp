/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 碰钉机器人主函数
 * @version 0.1
 * @date 2024-09-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <QApplication>
#include <string>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <QThread>
#include <QTextCodec>
#include <clocale>
#include <Eigen/Eigen>

#include "mainwindow.h"
#include "vision/VisionInterface.h"
#include "com/LaserDistanceBojke.h"
#include "cxxopts.hpp"

int line_detect_demo(int argc, char* argv[])
{
    std::string path = "E:\\ZBRobot\\Robot\\cache\\Image_20241011110413922.bmp";
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

int RunRobot(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("LNG Panel Loading Robot");
    w.show();
    return a.exec();
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
    // m_Com->SetLight(2, true);
    // m_Com->SetLight(3, true);
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
    m_stMeasuredata.m_LineDistance = {14.118399342577506,15.141133164344948,14.899653504249896,15.315110857435997,16.001300774736606,12.97208101039237};
    m_stMeasuredata.m_bLineDistance = std::vector<bool>(6, true);
    double line_dis_1 = ((m_stMeasuredata.m_LineDistance[0] - 15) * m_stMeasuredata.m_bLineDistance[0] - (m_stMeasuredata.m_LineDistance[1] - 15) * m_stMeasuredata.m_bLineDistance[1]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[0]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[1]));
    double line_dis_2 = ((m_stMeasuredata.m_LineDistance[2] - 15) * m_stMeasuredata.m_bLineDistance[2] - (m_stMeasuredata.m_LineDistance[3] - 15) * m_stMeasuredata.m_bLineDistance[3]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[2]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[3]));
    double line_dis_3 = ((m_stMeasuredata.m_LineDistance[4] - 15) * m_stMeasuredata.m_bLineDistance[4] - (m_stMeasuredata.m_LineDistance[5] - 15) * m_stMeasuredata.m_bLineDistance[5]) / (static_cast<int>(m_stMeasuredata.m_bLineDistance[4]) + static_cast<int>(m_stMeasuredata.m_bLineDistance[5]));

    std::cerr << "line_dis_1: " << line_dis_1 << "\n"
              << "line_dis_2: " << line_dis_2 << "\n"
              << "line_dis_3: " << line_dis_3 << "\n";

    return 0;
}

int main(int argc, char* argv[])
{
    std::map<std::string, std::function<int(int, char**)>> FunctionMap =
    {
        {"robot", RunRobot},
        {"line_demo", line_detect_demo},
        {"laser_demo", laserDemo},
        {"test_com", TestCom},
        {"TestTask", TestTask}
    };
    cxxopts::Options options("Robot", "zbrobot's project");
    options.add_options()("m,mode", "mode", cxxopts::value<std::string>()->default_value("robot"));
    std::string mode{ "robot" };
    try
    {
        auto result = options.parse(argc, argv);
        mode = result["mode"].as<std::string>();
    }
    catch (...)
    {
        std::cerr << "parse argument failed\n";
        return -1;
    }
    auto it = FunctionMap.find(mode);
    if (it != FunctionMap.end())
    {
        it->second(argc, argv);
    }
    else
    {
        std::cerr << "invalid argument: " << mode << "\n";
        return -1;
    }
}