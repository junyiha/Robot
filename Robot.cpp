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

#include "cxxopts.hpp"

#include "HMI/SceneSelectionWindow.hpp"
#include "vision/VisionInterface.h"
#include "com/LaserDistanceBojke.h"
#include "test/test.hpp"


int RunRobot(int argc, char* argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("LNG Panel Loading Robot");
    w.show();
    return a.exec();
}


int RunSceneSelect(int argc, char* argv[])
{
    QApplication app(argc, argv);

    APP::SceneSelectionWindow scene_selection_window;
    scene_selection_window.show();

    return app.exec();
}

int main(int argc, char* argv[])
{
    std::map<std::string, std::function<int(int, char**)>> FunctionMap =
    {
        {"robot", RunRobot},
        {"line_demo", line_detect_demo},
        {"laser_demo", laserDemo},
        {"test_com", TestCom},
        {"TestTask", TestTask},
        {"TestConfigManager", TestConfigManager},
        {"RunSceneSelect", RunSceneSelect}
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