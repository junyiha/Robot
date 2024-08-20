#include "Camera.h"
#include<chrono>




Camera::Camera(std::string cam_name, std::string rstp)
{
    this->camera_name = cam_name;
    this->rtsp = rstp;
}

//打开相机
void Camera::open_camera(){
    /**
 * @brief 打开相机
 *
 * 打开指定 RTSP 地址的相机，并设置缓存大小为 1。
 * 如果打开相机失败，输出错误信息并设置 isOpened 为 false；
 * 如果打开相机成功，输出成功信息并设置 isOpened 为 true。
 */
    this->cap.set(cv::CAP_PROP_BUFFERSIZE,1); //可设置缓存大小
    if(this->cap.open(this->rtsp)!=1){
        std::cout<<"camera:"<<this->camera_name<<" open failed"<<std::endl;
        this->isOpened = false;
    }else{
        std::cout<<"camera:"<<this->camera_name<<" open successed"<<std::endl;
        this->isOpened = true;
    }

}


void Camera::run() {
    /**
 * @brief 运行相机
 *
 * 启动相机并持续从相机中读取帧，直到线程控制标志被关闭。
 *
 * 如果相机已经在运行，则输出“重复运行”并返回。
 *
 * 在读取帧时，使用异步方式从视频源读取新的帧，并在满足一定条件后，通过`retrieve`函数从相机中取帧。
 *
 * 如果相机取帧失败，则输出“相机取帧失败”并返回。
 *
 * 读取到帧后，根据相机的旋转角度，对帧进行旋转操作。
 *
 * 最后，将旋转后的帧赋值给`img`成员变量，并处理Qt事件队列中的事件。
 *
 * @return 无返回值
 */


    if (running) {
        return;
    }
    this->running = true;


    while (this->thread_control_flag) {

        if (!this->cap.isOpened()) {
            return;
        }
        cv::Mat Camera_CImg;
        this->cap.grab(); //用于异步地从视频源读取新的帧，但不立即返回该帧  XXXXXXXX
        this->cap.retrieve(Camera_CImg); //相机取帧

        if (Camera_CImg.empty()) {
            std::cout << "Camera frame capture failed" << std::endl;
            return;
        }

//        std::string img_path = "E:/project/data/board_project/Cases/circle_cases/1234_hole1.jpg";
        std::string img_path = "E:/project/data/board_project/case_2/case_2/20240407214359.png";

        cv::Mat img = cv::imread(img_path);
        Camera_CImg = cv::Mat();
        Camera_CImg = img;
//        cv::resize(Camera_CImg,Camera_CImg,cv::Size(480,320));


        this->img_lock.lock();
        if (this->myQueue.size() < this->max_size) {
            this->myQueue.push(Camera_CImg);
        } else {
            this->myQueue.pop(); // 弹出队首元素
            this->myQueue.push(Camera_CImg); // 添加新元素
        }
        this->img_lock.unlock();
        QThread::msleep(200);

//        QApplication::processEvents();

    }

}

    //关闭相机
void Camera::stop()
{
        this-> thread_control_flag = false;
        this->running=false;
        this->isOpened = false;
        std::cout<<"stop is successed"<<std::endl;
}



Camera::~Camera(){
    this->stop();
}





