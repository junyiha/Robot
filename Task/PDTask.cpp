#include "PDTask.h"

namespace TASK
{

PDTask::PDTask()
{
    
}

PDTask::~PDTask()
{
    
}

bool PDTask::Parallel()
{
    std::clog << "执行 调平 指令...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::clog << "调平 指令执行成功！！！\n";

    return true;
}

bool PDTask::CheckFlatness()
{
    std::clog << "执行 检测平整度 指令...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::clog << "检测平整度指令执行成功！！！\n激光传感器偏差小于阈值，板壁距离满足调整要求，未检测到壁面\n";

    return true;
}

bool PDTask::Positioning()
{
    std::clog << "等待指令触发...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::clog << "收到定位指令，指令触发，开始定位！！！\n";

    return true;
}

bool PDTask::CheckLine()
{
    std::clog << "执行 检测边线 指令...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::clog << "检测边线指令执行成功！！！\n边线偏差小于阈值，末端调整数据合法\n";

    return true;
}

bool PDTask::MagentOn()
{
    std::clog << "执行 吸合 指令...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::clog << "吸合指令执行成功！！！\n";

    return true;
}

bool PDTask::AutoDoWeld()
{
    std::clog << "执行 自动碰钉 指令...\n";

    return true;
}

bool PDTask::CheckAutoDoWeld()
{
    std::clog << "正在执行 自动碰钉 指令...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::clog << "自动碰钉指令执行成功！！！\n";

    return true;
}

}  // namespace TASK