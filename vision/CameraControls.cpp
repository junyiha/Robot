#include "CameraControls.h"


CameraControls::CameraControls(std::string type){
    /**
     * @brief CameraControl 构造函数
     *
     * 根据传入的相机类型，创建并初始化相机对象。
     *
     * @param type 相机类型
     */

    std::map<std::string, std::string> rtsp_config;
    //相关配置参数
    if (type=="hole"){
        rtsp_config = {
                {"hole_1", "rtsp://192.168.1.111:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"hole_2", "rtsp://192.168.1.112:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"hole_3", "rtsp://192.168.1.113:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"hole_4", "rtsp://192.168.1.114:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
        };

        this->rotate_types = {
                    {"hole_1", 3},
                    {"hole_2", 3},
                    {"hole_3", 3},
                    {"hole_4", 3},
        };

    }else if(type=="edge"){
        rtsp_config = {
                {"edge_2", "rtsp://192.168.1.122:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"edge_4", "rtsp://192.168.1.124:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"edge_6", "rtsp://192.168.1.126:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"edge_7", "rtsp://192.168.1.127:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"edge_8", "rtsp://192.168.1.128:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"edge_9", "rtsp://192.168.1.129:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
                {"edge_10", "rtsp://192.168.1.130:554/user=admin_password=tlJwpbo6_channel=1_stream=0&amp;onvif=0.sdp?real_st"},
        };

        this->rotate_types = {

                        {"edge_2", 0},
                        {"edge_4", 0},
                        {"edge_6", 1},
                        {"edge_7", 0},
                        {"edge_8", 1},
                        {"edge_9", 0},
                        {"edge_10", 0},
         };
    }else{
        std::cout<<"当前仅支持边缘相机和圆孔相机,其它类型相机暂不支持!"<<std::endl;
        return ;
    }
    // 创建相机并进行启动
    int i=0;
    for (const auto& config : rtsp_config) {
        std::cout<<config.first<<"      "<<config.second<<std::endl;

        FFmpegCamera* cam_obj = new FFmpegCamera(config.second,config.first,this->rotate_types[config.first]);
        this->cameras[config.first] = cam_obj;
//        cam_obj->open_camera();
//        if(cam_obj->isOpened){
//            std::cout<<"carmear:"<<config.first<<" thread is running"<<std::endl;
//            cam_obj->start();
//        }
        cam_obj->connect();
        if(cam_obj->isConnect){
            cam_obj->start();
            i++;
        }

    }
    qDebug()<<"完成相机创建<<"<<i;
}

CameraControls::~CameraControls(){
    for ( auto& cam : this->cameras) {
        delete cam.second;
    }
}

void CameraControls::get_camera_data(){
    /**
   * @brief 获取相机数据
   *
   * 如果存在相机数据，则清空当前相机数据。
   * 遍历相机列表，为每个相机生成一个相机资源对象，并将其存储在相机数据映射中。
   *
   * 如果相机图像不为空，则将图像数据设置为相机资源对象的图像数据，并将状态设置为 true。
   * 如果相机图像为空，则将状态设置为 false。
   */
//   if(!this->cameras_data.empty()){
//       this->cameras_data.clear();
//   }
    std::map<std::string, CameraRes> temp_cameras_data;
    this->data_lock.lock();
    for (const auto& cam : this->cameras) {
        CameraRes c_res;
        c_res.cam_name = cam.first;
        unsigned nums = 0;
        cv::Mat temp_res;
        //相机启动时，可能存在抓取失败，或比较缓慢的现象
        while(true){
            if(cam.second->myQueue.size()>0){
                temp_res = cam.second->myQueue.back();
                break;
            }
            if(nums>50){break;}
            nums+=1;
        }

        if(!temp_res.empty()){
            c_res.img_data = temp_res;
            c_res.status = true;
        }else {
            c_res.status = false;
        }
        temp_cameras_data[cam.first] = c_res;
    }
    this->data_lock.unlock();
    this->cameras_data = temp_cameras_data;
}

CameraRes CameraControls::get_camera_data_by_name(std::string cam_name){
    /**
     * @brief 通过相机名称获取相机数据
     *
     * 根据给定的相机名称，从相机控制对象中检索相机数据。
     *
     * @param cam_name 相机名称
     *
     * @return 返回相机数据对象 CameraRes
     */
    CameraRes res;
    if (this->cameras.count(cam_name) > 0) {
        // key存在
        auto it = this->cameras.find(cam_name);
        cv::Mat value = it->second->myQueue.back();
        if(!value.empty()){
            res.img_data = value;
            res.status = true;
        }else{
            res.status = false;
        }
    } else {
        std::cout<<"camera:"+cam_name+"is not exists"<<std::endl;
        res.status = false;

    }
    return res;
}

void CameraControls::close_cameras(){
    /**
     * @brief 关闭相机
     *
     * 关闭所有相机并停止它们的运行。
     */
    for (auto& cam : this->cameras) {
        cam.second->stop();
        std::cout<<"camera:"+cam.first+"is closed!"<<std::endl;
    }

}

void CameraControls::close_cameras_by_name(std::string cam_name){
    /**
     * @brief 根据相机名称关闭相机
     *
     * 根据给定的相机名称，关闭相应的相机。
     *
     * @param cam_name 相机名称
     */
    if (this->cameras.count(cam_name) > 0) {
        FFmpegCamera* cam = this->cameras[cam_name];
        cam->stop();
        std::cout<<"camera:"+cam_name+"is closing!"<<std::endl;
    }else{
        std::cout<<"camera:"+cam_name+"is not exists!"<<std::endl;
        return ;
    }
}

