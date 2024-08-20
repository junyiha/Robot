#include"LidarHandler.h"

LidarHandler::LidarHandler(LidarHelper* lidar, GocatorControls* gocator, SharedData* sharedData){
    this->lidar_helper = lidar;
    this->gocator_controls = gocator;
    this->sharedData = sharedData;
}

LidarHandler::~LidarHandler(){
    this->detect_control_flag = false;
    this->thread_control_flag = false;
}


void LidarHandler::run(){

    while(this->thread_control_flag){
         QThread::sleep(100);

        if(this->detect_control_flag){


            // 获取数据
            this->gocator_controls->get_data();
            std::map<std::string, LidarData> temp_results; 
            std::map<std::string, cv::Mat> lidar_data = this->gocator_controls->lidars;
            for(auto& item : lidar_data) {
               LidarData lidar_res;
               if(item.second.empty()){ //判别是否获取到有效的点云数据
                   lidar_res.flag = false;
                   lidar_res.dist = -1;
                   lidar_res.gap = -1;
                   lidar_res.angle = -1;
                   lidar_res.show = cv::Mat();
                   lidar_res.error = "雷达未获取到有效的点云数据！";
                   temp_results[item.first] = lidar_res;
               }else{
                   Gocator *goc = this->gocator_controls->gocators[item.first];
                   this->lidar_helper->lidarDetecter(item.second, goc->isleft, goc->revAngle); //雷达成像检测
                   this->lidar_helper->shrink();
                   lidar_res.flag = this->lidar_helper->resultFlag;
                   lidar_res.dist = this->lidar_helper->lidarDist;
                   lidar_res.gap = this->lidar_helper->gap;
                   lidar_res.angle = this->lidar_helper->lidarAngle;
                   lidar_res.error = this->lidar_helper->error;
                   lidar_res.show = this->lidar_helper->show;
                   temp_results[item.first] = lidar_res;
                   this->lidar_helper->clear();
               }
            }

            QMutexLocker locker(&this->sharedData->mutex);
            if(this->results.size()>0){
                this->sharedData->spaceAvailable.wait(&this->sharedData->mutex);
            }
            this->results = temp_results;
            this->sharedData->dataReady.wakeOne();
        }
    }
}

void LidarHandler::is_start_detect(bool flag){
    this->detect_control_flag = flag;
}

void LidarHandler::closed(){
    this->thread_control_flag = false;
}




