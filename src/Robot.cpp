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
#include "hmi/SceneSelectionWindow.hpp"
#include "test/test.hpp"

static void InitLogger()
{
    //创建控制台日志记录器
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);

    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e][thread %t][%s:%#][%l]: %v");

    // 创建文件日志记录器: 滚动记录，最大文件5M，文件数量100个
    std::string log_path = ROOT_PATH;
    log_path += "logs/rotating.txt";
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path, 1048576 * 5, 100);

    //同步记录器，
    std::vector<spdlog::sink_ptr> sinks{ console_sink, rotating_sink };
    auto logger = std::make_shared<spdlog::logger>("logger", sinks.begin(), sinks.end());
    spdlog::register_logger(logger); //注册为全局日志，通过log_write访问;

    // 宏相关配置
    spdlog::set_default_logger(logger);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e][thread %t][%s:%#][%l]: %v");
    spdlog::set_level(spdlog::level::warn);
    spdlog::flush_every(std::chrono::seconds(3)); //每3s刷新一次
}

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
    InitLogger();
    std::map<std::string, std::function<int(int, char**)>> FunctionMap =
    {
        {"robot", RunRobot},
        {"line_demo", line_detect_demo},
        {"laser_demo", laserDemo},
        {"test_com", TestCom},
        {"TestTask", TestTask},
        {"TestConfigManager", TestConfigManager},
        {"RunSceneSelect", RunSceneSelect},
        {"TestTcpClient", TestTcpClient},
        {"TestFitBoard", TestFitBoard},
        {"TestRobot", TestRobot}
    };
    cxxopts::Options options("Robot", "boarding robot's project");
    options.add_options()("m,mode", "run mode", cxxopts::value<std::string>()->default_value("RunSceneSelect"));
    options.add_options()("v,verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"));
    try
    {
        auto result = options.parse(argc, argv);

        if (result["verbose"].as<bool>())
        {
            spdlog::set_level(spdlog::level::debug);
        }
        auto it = FunctionMap.find(result["mode"].as<std::string>());
        if (it != FunctionMap.end())
        {
            return it->second(argc, argv);
        }
        else
        {
            SPDLOG_ERROR("invalid argument: ", result["mode"].as<std::string>());
            return -1;
        }
    }
    catch (...)
    {
        SPDLOG_ERROR("parse argument failed");
        return -1;
    }
}