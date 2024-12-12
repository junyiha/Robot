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
#include "Network/TcpClient.hpp"

void line_detect_demo()
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
}

void laserDemo()
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
}

int RunRobot(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("LNG Panel Loading Robot");
    w.show();
    return a.exec();
}

int TestBoostAsio()
{
    Network::TcpClient tcp_client;
    while (true)
    {
        std::cerr << "input command: \n";
        std::string cmd;
        std::cin >> cmd;
        if (cmd == "connect")
        {
            tcp_client.ConnectToServer("180.101.50.188", 80);
        }
        else if (cmd == "disconnect")
        {
            tcp_client.Disconnect();
        }
        else if (cmd == "recv")
        {
            std::vector<char> buf(1 * 1024);
            std::size_t recv_size = tcp_client.RecvData(buf);
            if (recv_size == 0)
            {
                std::cerr << "failed to receive data, error message: " << tcp_client.error_code.message() << "\n";
            }
            else
            {
                std::cerr << "receive message: " << std::string(buf.data(), buf.size()) << "\n";
            }
        }
        else
        {
            std::cerr << "invalid command: " << cmd << "\n";
        }

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(200ms);
    }
    return 0;
}

int main(int argc, char* argv[])
{
    return RunRobot(argc, argv);

    // line_detect_demo();
    // laserDemo();
    // return 0;
}