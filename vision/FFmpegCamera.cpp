#include "FFmpegCamera.h"
#include <QImage>


cv::Mat QImageToMat(QImage& srcQImage) {
    // 确保 QImage 是以 Format_RGB32 格式存储的
    std::cout<<"format:"<<srcQImage.format()<<QImage::Format_RGB32<<std::endl;
    if (srcQImage.format() != QImage::Format_RGB32) {
        srcQImage = srcQImage.convertToFormat(QImage::Format_RGB32);
    }

    // 获取 QImage 的像素数据和尺寸
    const uchar* srcData = srcQImage.bits();
    int width = srcQImage.width();
    int height = srcQImage.height();

    // 创建一个与 QImage 尺寸相同的 Mat 对象
    // 注意：OpenCV 的图像矩阵行是自上而下的，所以行数是 height，列数是 width
    cv::Mat dstMat(height, width, CV_8UC4, (void*)srcData);

    // QImage 的 Format_RGB32 是以 ARGB 的形式存储的，而 OpenCV 的 CV_8UC4 也是以这个顺序存储的
    // 但是 OpenCV 默认使用 BGR 格式，所以我们需要将颜色通道从 RGB 转换为 BGR
//    cv::cvtColor(dstMat, dstMat, cv::COLOR_RGBA2BGRA);

    // 如果你不需要保留 alpha 通道，可以进一步从 BGRA 转换到 BGR
    // cv::cvtColor(dstMat, dstMat, cv::COLOR_BGRA2BGR);

    return dstMat;
}


void image_correction(cv::Mat& img, unsigned rotate_type){
    //图像方向矫正

    cv::Mat tmp2;
    switch (rotate_type) {
      case 1://顺时针90
        cv::transpose(img, tmp2);
        cv::flip(tmp2, img, 1);
        break;
      case 2: //顺时针180°
        cv::flip(img, tmp2, -1);
        img = tmp2;
        break;
      case 3: //逆时针90
        cv::transpose(img, tmp2);
        cv::flip(tmp2, img, 0);
        break;
    }
}


FFmpegCamera::FFmpegCamera(std::string rtsp, std::string cam_name, unsigned rotate_type){
    this->rtsp = rtsp;
    this->camera_name = cam_name;
    this->rotate_type = rotate_type;

}


void FFmpegCamera::connect(){

    pFormatCtx = avformat_alloc_context();  //init FormatContext
    //初始化FFmpeg网络模块
    avformat_network_init();    //init FFmpeg network

    av_dict_set(&avdic,"timeout","500000",0);
    //open Media File
    ret = avformat_open_input(&pFormatCtx,this->rtsp.c_str(),NULL,&avdic);

    if(ret != 0){
        av_strerror(ret,errors,sizeof(errors));
        qDebug() <<"Failed to open video: ["<< ret << "]"<< errors << endl;
        this->isConnect = false;
        return ;
    }

    //Get audio information
    ret = avformat_find_stream_info(pFormatCtx,NULL);
    if(ret != 0){
        av_strerror(ret,errors,sizeof(errors));
        qDebug() <<"Failed to get audio info: ["<< ret << "]"<< errors << endl;
        this->isConnect = false;
        return ;
    }
    //循环查找视频中包含的流信息，直到找到视频类型的流
    //便将其记录下来 videoIndex
    //这里我们现在只处理视频流  音频流先不管他
    for (i = 0; i < pFormatCtx->nb_streams; i++) {
        if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            videoIndex = i;
        }
    }

    //如果videoIndex为-1 说明没有找到视频流
    if (videoIndex == -1) {
        printf("Didn't find a video stream.\n");
        this->isConnect = false;
        return ;
    }
    //配置编码上下文，AVCodecContext内容
    //1.查找解码器
    pCodec = avcodec_find_decoder(pFormatCtx->streams[videoIndex]->codecpar->codec_id);
    //2.初始化上下文
    pCodecCtx = avcodec_alloc_context3(pCodec);
    //3.配置上下文相关参数
    avcodec_parameters_to_context(pCodecCtx,pFormatCtx->streams[videoIndex]->codecpar);
    //4.打开解码器
    ret = avcodec_open2(pCodecCtx, pCodec, NULL);
    if(ret != 0){
        av_strerror(ret,errors,sizeof(errors));
        qDebug() <<"Failed to open Codec Context: ["<< ret << "]"<< errors << endl;
        this->isConnect = false;
        return ;
    }
    //初始化视频帧
    pFrame = av_frame_alloc();
    pFrameRGB = av_frame_alloc();
    //为out_buffer申请一段存储图像的内存空间
    out_buffer = (unsigned char*)av_malloc(av_image_get_buffer_size(AV_PIX_FMT_RGB32,pCodecCtx->width,pCodecCtx->height,1));
    //实现AVFrame中像素数据和Bitmap像素数据的关联
    av_image_fill_arrays(pFrameRGB->data,pFrameRGB->linesize, out_buffer,
                   AV_PIX_FMT_RGB32,pCodecCtx->width, pCodecCtx->height,1);
    //为AVPacket申请内存
    packet = (AVPacket *)av_malloc(sizeof(AVPacket));
    //打印媒体信息
    av_dump_format(pFormatCtx,0,this->rtsp.c_str(),0);
    //初始化一个SwsContext
    img_convert_ctx = sws_getContext(pCodecCtx->width, pCodecCtx->height,
                pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height,
                AV_PIX_FMT_RGB32, SWS_BICUBIC, NULL, NULL, NULL);
    av_new_packet(packet, pCodecCtx->width*pCodecCtx->height);
    this->isConnect = true;

}

void FFmpegCamera::run(){

    if(!this->isConnect){
        qDebug() <<"the connect of ffmpeg is failed!"<< endl;
        return ;
    }


    while(this->thread_control_flag){

        if(av_read_frame(pFormatCtx,packet) >=0){
            //判断视频帧
            if(packet->stream_index == videoIndex){
                //解码视频帧
                ret = avcodec_send_packet(pCodecCtx, packet);
                if(ret<0){
                    usleep(1000);
                    printf("decode error\n");
                    continue;
                }
                ret = avcodec_receive_frame(pCodecCtx, pFrame);
                if(ret != 0){
                    av_strerror(ret,errors,sizeof(errors));
                    qDebug() <<"Failed to decode video frame: ["<< ret << "]"<< errors << endl;
                    usleep(1000);
                    printf("***************receive frame error\n");
                    continue;
                }

                if (ret == 0) {
                    //处理图像数据
                    sws_scale(img_convert_ctx,
                                            (const unsigned char* const*) pFrame->data,
                                            pFrame->linesize, 0, pCodecCtx->height, pFrameRGB->data,
                                            pFrameRGB->linesize);
                    cv::Mat img(pFrame->height, pFrame->width, CV_8UC4,(uchar*)pFrameRGB->data[0]);
                    cv::cvtColor(img, img, cv::COLOR_RGBA2BGR);
                    if(!img.empty()){
                        image_correction(img,this->rotate_type);
                    }

                    this->img_lock.lock();
                    if (this->myQueue.size() < this->max_size) {
                       this->myQueue.push(img);
                    } else {
                       this->myQueue.pop(); // 弹出队首元素
                       this->myQueue.push(img); // 添加新元素
                    }
                     this->img_lock.unlock();

                    QThread::usleep(200);

                }

            }

            //释放packet空间
//            av_free_packet(packet);
            av_packet_unref(packet);
            memset(out_buffer,0, sizeof(out_buffer));
        }
    }


    // 释放资源
    av_free(out_buffer);
    av_free(pFrameRGB);
    avcodec_close(pCodecCtx);
    if(pCodecCtx!=NULL){
        avcodec_free_context(&pCodecCtx);
        avdic= NULL;
    }

   if(pFormatCtx!=NULL){
       avformat_close_input(&pFormatCtx);
       pFormatCtx = NULL;
   }
}


void FFmpegCamera::destroy(){
    av_free(out_buffer);
    av_free(pFrameRGB);

    sws_freeContext(img_convert_ctx);
    avcodec_close(pCodecCtx);
    avcodec_free_context(&pCodecCtx);
    avformat_close_input(&pFormatCtx);
}


FFmpegCamera::~FFmpegCamera(){
    this->destroy();
    this->stop();
    std::cout<<"ffmpeg is destroy!"<<std::endl;
}

void FFmpegCamera::stop()
{
    this-> thread_control_flag = false;
    this->isConnect=false;
    // 释放资源
    av_free(out_buffer);
    av_free(pFrameRGB);
    avcodec_close(pCodecCtx);
    if(pCodecCtx!=NULL){
        avcodec_free_context(&pCodecCtx);
        avdic= NULL;
    }

   if(pFormatCtx!=NULL){
       avformat_close_input(&pFormatCtx);
       pFormatCtx = NULL;
   }
    std::cout<<"stop is successed"<<std::endl;
}




