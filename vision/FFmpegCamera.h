#include <iostream>
#include <string>
#include <QThread>
#include <QMutex>
#include <opencv2/opencv.hpp>
#include <QtDebug>
#include<QTime>
#include <QCoreApplication>

extern "C"
{
//avcodec:编解码(最重要的库)
#include <libavcodec/avcodec.h>
//avformat:封装格式处理
#include <libavformat/avformat.h>
//swscale:视频像素数据格式转换
#include <libswscale/swscale.h>
//avdevice:各种设备的输入输出
#include <libavdevice/avdevice.h>
//avutil:工具库（大部分库都需要这个库的支持）
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
}


class FFmpegCamera: public QThread{

public:
    std::string rtsp;
    std::string camera_name; //相机名称

    //进程运行状态参数
    bool isConnect=false; //相机是否打开
    bool running=false;
    bool thread_control_flag= true;
    std::queue<cv::Mat> myQueue;
    unsigned max_size = 5;
    unsigned rotate_type=0;
    QMutex img_lock;

    // ffmpeg相关配置
    AVFormatContext *pFormatCtx = NULL;
    AVCodecContext *pCodecCtx = NULL;
    const AVCodec *pCodec = NULL;
    AVFrame *pFrame,*pFrameRGB;
    AVPacket *packet;
    AVDictionary *avdic=NULL;
    struct SwsContext *img_convert_ctx;

    unsigned char *out_buffer;
    int i,videoIndex;
    int ret;
    char errors[1024] = "";


    void connect();
    void run();
    void stop();
    void destroy();
    FFmpegCamera(std::string rstp, std::string cam_name, unsigned rotate_type);
    ~FFmpegCamera();

};
