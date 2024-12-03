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
#include "mainwindow.h"
#include <QApplication>
#include <string>
#include "vision/VisionInterface.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <QThread>
#include <QTextCodec>
#include <clocale>
#include <Eigen/Eigen>
#include "com/LaserDistanceBojke.h"

#include "cxxopts.hpp"

void initLog()
{
    // 创建控制台日志记录器
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S:%e] [%^%l%$] %v");

    // 创建文件日志记录器: 滚动记录，最大文件5M，文件数量100个
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("logs/rotating.txt", 1048576 * 5, 100);

    // 同步记录器，
    std::vector<spdlog::sink_ptr> sinks{console_sink, rotating_sink};
    auto logger = std::make_shared<spdlog::logger>("logger", sinks.begin(), sinks.end());

    spdlog::register_logger(logger);              // 注册为全局日志，通过log_write访问;
    spdlog::flush_every(std::chrono::seconds(3)); // 每3s刷新一次
    // 根据需要调整记录级别：调试debug，发布info
    spdlog::set_level(spdlog::level::debug);
    std::shared_ptr<spdlog::logger> log = spdlog::get("logger");
}

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

    const char *port = "COM2";
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

int RunRobot(int argc, char *argv[])
{
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8")); // 设置中文编码
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("LNG Panel Loading Robot");
    w.show();
    return a.exec();
}

int main(int argc, char *argv[])
{
    return RunRobot(argc, argv);

    // line_detect_demo();
    // laserDemo();
    // return 0;
}