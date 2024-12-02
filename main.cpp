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

#include "SceneSelectionWindow.hpp"

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

void vision_demo()
{
    initLog();
    VisionInterface vision;
    vision.start();

    VisionResult res = vision.getVisResult();
    if (res.status)
    {
        for (int i = 0; i < 5; ++i)
        {
            std::cout << "cam:" << i << " :dist:" << res.stData.m_LineDistance[i] << std::endl;
        }
    }
    QThread::sleep(1000 * 30);
}

void line_detect_demo()
{

    std::string path = "../cache/cam_6_2024-11-01-17-28-35.png";
    cv::Mat img = cv::imread(path);
    LineDetector line_tool;
    auto start = std::chrono::high_resolution_clock::now();
    auto res = line_tool.getLinesDistance(img);
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

int RunRobot(int argc, char *argv[])
{
    QApplication app(argc, argv);

    APP::SceneSelectionWindow scene_selection_window;
    scene_selection_window.show();

    return app.exec();
}

int main(int argc, char *argv[])
{
    return RunRobot(argc, argv);

    //      line_detect_demo();
}