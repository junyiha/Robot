//
// Created by chen on 2024/7/6.
//

#include "VisionInterface.h"

VisionInterface::VisionInterface()
{


    this->camera_controls = new CameraManager();  // 相机管理类
    this->line_helper = new LineDetector();       // 边线检测功能类
    this->sharedDataLine = new SharedData();      // 共享数据类

    this->laser_controls = new LaserScanerControls();
    this->lidar_helper = new LidarHelper();
    this->sharedDataLaser = new SharedData();


    // 相机读取+边线检测 端到端处理类(线程)
    this->line_handler = new LineDetectorRunner(this->line_helper, this->camera_controls, this->sharedDataLine);
    this->lidar_handler = new LidarHandler(this->lidar_helper, this->laser_controls, this->sharedDataLaser);
    this->line_handler->start();
    this->lidar_handler->start();


    this->scales_line = {
            {"LineCam_1", 0.275862069},
            {"LineCam_2", 0.275229358},
            {"LineCam_3", 0.275862069},
            {"LineCam_4", 0.274914089},
            {"LineCam_5", 0.274914089},
            {"LineCam_6", 0.274509804},
    };

    this->camera_offset = {
            {"LineCam_1", 83},
            {"LineCam_2", 125},
            {"LineCam_3", 88},
            {"LineCam_4", 120},
            {"LineCam_5", 132},
            {"LineCam_6", 127},
    };
    //    this->camera_offset = {
    //            {"LineCam_1", 95},
    //            {"LineCam_2", 113},
    //            {"LineCam_3", 89},
    //            {"LineCam_4", 113},
    //            {"LineCam_5", 65},
    //            {"LineCam_6", 98},
    //    };

    this->logger = spdlog::get("logger");
}

VisionInterface::~VisionInterface()
{

    if (this->line_handler != nullptr)
        delete this->line_handler;
    if (this->camera_controls != nullptr)
        delete this->camera_controls;
    if (this->sharedDataLine != nullptr)
        delete this->sharedDataLine;

    if (this->line_handler != nullptr)
    {
        delete this->line_handler;
    }
    if (this->laser_controls != nullptr)
    {
        delete this->laser_controls;
    }
    if (this->sharedDataLaser != nullptr)
    {
        delete this->sharedDataLaser;
    }

    this->isRunning = false;
    this->quit();
}

void VisionInterface::getDetectResult(VisionResult& vis_result)
{

    //    QMutexLocker locker_line(&this->sharedDataLine->mutex);
    //    if(this->line_handler->results.size()>0){
    //        this->line_handler->results.clear();
    //    }
    //    this->sharedDataLine->spaceAvailable.wakeOne();
    //    if(!this->line_handler->results.size()){
    //        this->sharedDataLine->dataReady.wait(&this->sharedDataLine->mutex);
    //    }

        // 获取直线检测结果
    std::map<std::string, LineDetectRes> lineDetectRes;
    lineDetectRes = this->line_handler->getDetectResults();
    if (lineDetectRes.size() > 0)
    {
        this->line_res = lineDetectRes;
        this->parser_result("line", &vis_result.stData);
        vis_result.lineStatus = true;
    }
    else
    {
        vis_result.lineStatus = false;
    }

    // 获取轮廓激光测量结果
    std::map<std::string, LidarData> lidarDetectRes;
    this->pointMaskMutex.lock();
    lidarDetectRes = this->lidar_handler->getLaserDetectResults();
    this->pointMaskMutex.unlock();
    if (lidarDetectRes.size() > 0)
    {
        this->lidar_res = lidarDetectRes;
        this->parser_result("laser", &vis_result.stData);
        vis_result.laserStatus = true;
    }
    else
    {
        vis_result.laserStatus = false;
    }


    // 轮廓激光结果与视觉测量结果协同
    // for (int i = 0; i < 4; i++)
    // {
    //     if (vis_result.stData.m_bLaserProfile[i])
    //     { // 如果轮廓激光数据有效
    //         if (vis_result.stData.m_bLineDistance[layer2camera[i]])
    //         { // 轮廓激光对应的相机位置有效
    //             if (abs(vis_result.stData.m_LineDistance[layer2camera[i]] - vis_result.stData.m_LaserGapDistance[i] + 15) < 5)
    //             {
    //                 vis_result.stData.m_bLineDistance[layer2camera[i]] = true;
    //                 vis_result.stData.m_LineDistance[layer2camera[i]] = vis_result.stData.m_LaserGapDistance[i] - 15;
    //             }
    //             else
    //             {
    //                 // 有待于进一步观察
    //             }
    //         }
    //         else
    //         { // 相机测量数据无效
    //             vis_result.stData.m_bLineDistance[layer2camera[i]] = true;
    //             vis_result.stData.m_LineDistance[layer2camera[i]] = vis_result.stData.m_LaserGapDistance[i] - 15;
    //         }
    //     }
    // }
}

void VisionInterface::parser_result(std::string paserType, stMeasureData* stm)
{

    if (paserType == "line")
    {
        if (this->line_res.size() > 0)
        {
            for (auto& line : this->line_res)
            {
                std::string prefix = line.first;
                size_t index = prefix.find("_");
                int number = prefix[index + 1] - '0';
                if (index + 1 != prefix.size() - 1)
                {
                    number = (prefix[index + 1] - '0') * 10 + (prefix[index + 2] - '0');
                }
                //                logger->info("********************{} pixel distance: {}**************************", line.first,line.second.dist);

                double dist = line.second.dist * this->scales_line[line.first];
                //                double dist = (line.second.dist-this->camera_offset[line.first]);
                //                if(number-1 == 4){
                //                    dist = 0;
                //                }
                if (dist < 1)
                {
                    stm->m_LineDistance[number - 1] = 0;
                    stm->m_bLineDistance[number - 1] = false;
                }
                else
                {
                    stm->m_LineDistance[number - 1] = dist;
                    stm->m_bLineDistance[number - 1] = line.second.status;
                }
            }
        }
    }

    if (paserType == "laser")
    {
        if (this->lidar_res.size() > 0)
        {
            for (auto& lidar : this->lidar_res)
            {
                std::string prefix = lidar.first;
                size_t index = prefix.find("_");
                int number = prefix[index + 1] - '0';
                stm->m_LaserGapHeight[number - 1] = lidar.second.dist; // 板高差
                stm->m_LaserGapDistance[number - 1] = lidar.second.gap; // 板间距
                stm->m_bLaserProfile[number - 1] = lidar.second.flag;  // 板间测量结果有效性
            }
        }
    }
}

VisionResult VisionInterface::getVisResult()
{
    VisionResult vis_result;
    memset(&vis_result, 0, sizeof(vis_result));
    this->mutex.lock();
    vis_result = this->vis_result_;
    this->mutex.unlock();
    return vis_result;
}


// 视觉检测结果线程
void VisionInterface::run()
{

    while (this->isRunning)
    {
        if (this->is_Detected)
        {
            // 获取检测结果
            memset(&this->vis_result, 0, sizeof(this->vis_result));
            getDetectResult(this->vis_result);
            // 结果拷贝
            this->mutex.lock();
            this->vis_result_ = this->vis_result;
            this->mutex.unlock();
        }
        QThread::msleep(200);
    }
}


std::map<std::string, LineDetectRes> VisionInterface::getLineRes()
{
    std::map<std::string, LineDetectRes> line_res;
    this->mutex.lock();
    line_res = this->line_res;
    this->mutex.unlock();
    return line_res;
}


void VisionInterface::closeThread()
{
    // 关闭相机线程
    this->camera_controls->closeAllCameraThread();

    // 关闭LineDetectorRunner线程
    this->line_handler->closedThread();

    // 关闭自身线程
    this->isRunning = false;
    QThread::quit();


}