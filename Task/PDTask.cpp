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
    std::clog << "执行调平指令...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::clog << "调平指令执行成功！！！\n";

    return true;
}

}  // namespace TASK