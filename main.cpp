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
void initLog()
{
    //创建控制台日志记录器
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S:%e] [%^%l%$] %v");

    // 创建文件日志记录器: 滚动记录，最大文件5M，文件数量100个
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("logs/rotating.txt", 1048576 *5 , 100);

    //同步记录器，
    std::vector<spdlog::sink_ptr> sinks {console_sink, rotating_sink};
    auto logger = std::make_shared<spdlog::logger>("logger", sinks.begin(), sinks.end());

    spdlog::register_logger(logger); //注册为全局日志，通过log_write访问;
    spdlog::flush_every(std::chrono::seconds(3)); //每3s刷新一次
    //根据需要调整记录级别：调试debug，发布info
    spdlog::set_level(spdlog::level::debug);
    std::shared_ptr<spdlog::logger> log = spdlog::get("logger");

}





void vision_demo(){
    initLog();
    VisionInterface vision;
    vision.start();

    VisionResult res = vision.getVisResult();

    if(res.status){
        for(int i=0;i<5;++i){
            std::cout<<"cam:"<<i<<" :dist:"<<res.stData.m_LineDistance[i]<<std::endl;
        }
    }

    QThread::sleep(1000*30);

}


#include<Eigen/Eigen>
int main(int argc, char *argv[])
{
//    QTextCodec::setCodecForTr(QTextCodec::codecForName("UTF-8"));
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8")); // 设置中文编码

    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("LNG Panel Loading Robot");
    w.show();

    QMessageBox::information(nullptr, "Info", "请确保机器人底部升降电机位于初始位置");
    return a.exec();

}

