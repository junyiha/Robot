#include"LidarHandler.h"

LidarHandler::LidarHandler(LidarHelper* lidarHelper, LaserScanerControls* laserControls, SharedData* sharedData)
{
    this->lidar_helper = lidarHelper;
    this->laser_controls = laserControls;
    this->sharedData = sharedData;
}

LidarHandler::~LidarHandler()
{
    this->detect_control_flag = false;
    this->thread_control_flag = false;
}


void LidarHandler::run()
{
    static int cout = 0;
    while (this->thread_control_flag)
    {

        if (this->detect_control_flag)
        {
            if (!laser_controls)
            {
                continue;
            }
            // 获取数据
            this->laser_controls->getDataAll();


            std::map<std::string, cv::Mat> lidar_data = this->laser_controls->pointCloudMasks;
            std::map<std::string, LidarData> temp_results;
            for (auto& item : lidar_data)
            {
                LidarData lidar_res;
                if (item.second.empty())
                { //判别是否获取到有效的点云数据
                    lidar_res.flag = false;
                    lidar_res.dist = -1;
                    lidar_res.gap = -1;
                    lidar_res.angle = -1;
                    lidar_res.show = cv::Mat();
                    lidar_res.error = "雷达未获取到有效的点云数据！";
                    temp_results[item.first] = lidar_res;
                }
                else
                {
                    MyBestfitLaserScaner* scaner_ = this->laser_controls->scaners[item.first];
                    // auto begin = std::chrono::system_clock::now();

                    this->lidar_helper->lidarDetecter(item.second, scaner_->isleft, scaner_->revAngle, scaner_->minX, scaner_->minY, this->laser_controls->direct_config[item.first]); //雷达成像检测
                    // auto duration = std::chrono::system_clock::now() - begin;
                    // SPDLOG_INFO("Lidar duration: {}  ms", std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());

                    this->lidar_helper->shrink();


                    //                   if(this->lidar_helper->resultFlag){
                    //                        std::string img_path = "E:\\projects\\zjy\\Robot-zb\\out\\" + std::to_string(cout)+".png";
                    //                       cv::imwrite(img_path, item.second);
                    //                       cout++;
                    //                   }
                    lidar_res.flag = this->lidar_helper->resultFlag;
                    lidar_res.dist = this->lidar_helper->lidarDist;
                    lidar_res.gap = this->lidar_helper->gap;
                    lidar_res.angle = this->lidar_helper->lidarAngle;
                    lidar_res.error = this->lidar_helper->error;
                    lidar_res.show = this->lidar_helper->show;
                    lidar_res.pointValidFlag = this->lidar_helper->vertexValidFlag; // 绝缘板角点标志位
                    lidar_res.board = this->lidar_helper->vertex;   // 绝缘板角点坐标
                    temp_results[item.first] = lidar_res;
                    this->lidar_helper->clear();
                }
            }

            //            QMutexLocker locker(&this->sharedData->mutex);
            this->pointsMaskMutex.lock();
            this->results = temp_results;
            this->pointsMaskMutex.unlock();

            //            if(this->results.size()>0){
            //                this->sharedData->dataReady.wakeOne();
            //                this->sharedData->spaceAvailable.wait(&this->sharedData->mutex);
            //            }
        }
        Sleep(200);
    }
}

void LidarHandler::is_start_detect(bool flag)
{
    this->detect_control_flag = flag;
}

void LidarHandler::closed()
{
    this->thread_control_flag = false;
}

std::map<std::string, LidarData> LidarHandler::getLaserDetectResults()
{

    std::map<std::string, LidarData> results;
    this->pointsMaskMutex.lock();
    if (this->results.size() > 0)
    {
        results = this->results;
    }
    this->pointsMaskMutex.unlock();
    return results;
}




