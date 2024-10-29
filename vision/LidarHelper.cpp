//
// Created by csh_i on 2024/5/6.
//

#include "LidarHelper.h"

void get_bin_img(cv::Mat img, cv::Mat &bin_img) {
    /**
  * @brief ��ȡ��ֵ��ͼ��
  *
  * ��������ͼ����ж�ֵ�����������ض�ֵ�����ͼ��
  *
  * @param img ����ͼ��
  * @param bin_img �����ֵ��ͼ��
  */
    cv::Mat element;//����
    element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat dstImage;
    cv::dilate(img, img, element);

    cv::Mat imageGray;
    cvtColor(img, imageGray, cv::COLOR_BGR2GRAY);
    threshold(imageGray, bin_img, 55, 200, cv::THRESH_BINARY);

}


void get_connected_components_info(cv::Mat imageBW, cv::Mat &outImage, cv::Mat &stats, cv::Mat &centroids,  cv::Mat &show_rect, std::vector<int> &linesIndex){
/**
 * @brief ��ȡ��ͨ������Ϣ
 *
 * �Ӷ�ֵ��ͼ���л�ȡ��ͨ������Ϣ��������ÿ����ͨ��������ĺ���Ӿ��Ρ�
 *
 * @param imageBW ��ֵ��ͼ��
 * @param outImage ���ͼ��
 * @param stats ��ͨ����ͳ����Ϣ
 * @param centroids ��ͨ������������
 * @param show_rect ������Ӿ��ε�ͼ��
 * @param linesIndex ��ͨ������������
 */

    int count = cv::connectedComponentsWithStats(imageBW, outImage, stats, centroids, 8, CV_16U);
    if(count<=0){
        return;
    }

    // Ϊÿ����ͨ������һ�������ɫ
    cv::RNG rng(time(NULL));
    std::vector<cv::Vec3b> colors;
    for (int i = 0; i < count; i++) {
        cv::Vec3b vec3 = cv::Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        colors.push_back(vec3);
    }


    cv::Mat dst = cv::Mat::zeros(imageBW.size(), imageBW.type());
    for (int i = 1; i < count; i++) {
        //�ҵ���ͨ�������
        int center_x = centroids.at<double>(i, 0);
        int center_y = centroids.at<double>(i, 1);

        //���εĵ�ͱ�
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        if(area<50){
            continue;
        }
        linesIndex.push_back(i);
        //�������ĵ�
        cv::circle(show_rect, cv::Point(center_x, center_y), 2, cv::Scalar(0, 255, 0), 2, 8, 0);
        //��Ӿ���
        cv::Rect rect(x, y, w, h);
        cv::rectangle(show_rect, rect, cv::Scalar(255,255,0), 2, 8, 0);
        //std::cout << "count:" << i << "  area:" << area << std::endl;
    }



}


std::vector<std::vector<cv::Point2f>>  get_lines_by_connected_components(std::vector<int> linesIndex, cv::Mat img, cv::Mat outImage) {
    /**
    * @brief ������ͨ�����ȡֱ����Ϣ
    *
    * ���ݸ�������ͨ����������ԭʼͼ������ͼ����ȡÿ����ͨ�����ֱ����Ϣ��������ֱ�ߵĶ˵����ꡣ
    *
    * @param linesIndex ��ͨ��������
    * @param img ԭʼͼ��
    * @param outImage ���ͼ��
    *
    * @return ֱ�ߵĶ˵����꼯��
    */
    //------------���ֱ��--------------
    //������ͨ��ͼ���ͼ
    std::vector<cv::Mat> linesImg;
    for(auto name : linesIndex){
        cv::Mat result = cv::Mat::zeros(img.size(), img.type());
        for (int i = 0; i < img.rows; ++i) {
            for (int j = 0; j < img.cols; ++j) {
                int label = outImage.at<uint16_t>(i, j);
                if (label == name) {
                    result.at<cv::Vec3b>(i, j) =cv::Vec3b(255,255,255);
                }
            }
        }
        linesImg.push_back(result);
    }


    //���ֱ��
    std::vector<std::vector<cv::Point2f>> linesEnd;
    for(auto img: linesImg){

        int img_width = img.cols;
        int img_height = img.rows;

        cv::Mat gray_image,bool_image;
        cv::cvtColor(img,gray_image,cv::COLOR_BGR2GRAY);
        cv::threshold(gray_image,bool_image,0,255,cv::THRESH_OTSU);
        //��ȡ��ά�㼯
        float minX=1000000,maxX=0;
        std::vector<cv::Point> point_set;
        cv::Point point_temp;
        for( int i = 0; i < img_height; ++i)
        {
            for( int j = 0; j < img_width; ++j )
            {
                if (bool_image.at<unsigned char>(i,j) >= 255)
                {
                    minX=j<minX?j:minX;
                    maxX=j>maxX?j:maxX;
                    point_temp.x = j;
                    point_temp.y = i;
                    point_set.push_back(point_temp);
                }
            }
        }


        cv::Vec4f fitline;
        //��Ϸ���������С���˷�
        cv::fitLine(point_set,fitline,cv::DIST_L2,0,0.01,0.01);

        //���ֱ���ϵ�������
        double k_line = fitline[1]/fitline[0];
        cv::Point p1(minX,k_line*(minX - fitline[2]) + fitline[3]);
        cv::Point p2(maxX,k_line*(maxX - fitline[2]) + fitline[3]);

        std::vector<cv::Point2f> end={p1,p2};
        linesEnd.push_back(end);

    }
    return linesEnd;
}


LidarHelper::LidarHelper()
{
    this->resultFlag = false;
}

void LidarHelper::lidarDetecter(cv::Mat img,bool isleft,bool revAngle, \
                                int min_X, int min_Y, bool pIsLeft)
{
    /* min_X: ���ƽ������Сxֵ
     * min_y: ���ƽ�������xֵ
     * direct_b: ��ʾ���߱�Ե�㷽��
     * direct_r: ��ʾ�ο����Ե�㷽��
     */

    this->Left=isleft;
    //------------�õ���ͨ��ɸѡ���--------------
    if(img.empty()){
        img = cv::imread("D:/_Project/Ship/program/ZBRobot/ZBRobotV23/bin/board/Lidar_test.jpg");
        error="no image";
        std::cout<<error<<std::endl;
    }
//    if (img.empty()) {
//        std::cout << "not get data" << std::endl;
//        error="û�н��ܵ�ͼ��";
//        std::cout<<"û�н��ܵ�ͼ��"<<std::endl;
//        return;
//    }
    this->show=img.clone();

    if(img.channels()<3){//ת��ɫ
        cv::Mat color_show;
        cv::cvtColor(img, color_show, cv::COLOR_GRAY2BGR);
        img=color_show;
    }
    // ��ֵ��
    cv::Mat imageBW;
    get_bin_img(img, imageBW);


    //������ͨ����,ɸѡ��������������ͨ��
    cv::Mat outImage, stats, centroids;
    cv::Mat show_rect;
    img.copyTo(show_rect);
    std::vector<int> linesIndex;
    get_connected_components_info(imageBW, outImage, stats, centroids,  show_rect, linesIndex);
    this->show=show_rect;
    cv::Mat shrink;
    cv::resize(img, shrink, cv::Size(900,900), 0, 0, cv::INTER_AREA);

    //ֱ�����
    std::vector<std::vector<cv::Point2f>> linesEnd= get_lines_by_connected_components(linesIndex, img, outImage);

    //------------��ȡ�����ǵ�λ����Ϣ, �û��Զ�ȡ��ģ��λ�˸�֪--------------
    if(linesEnd.size()==1){

        double dx = linesEnd[0][0].x - linesEnd[0][1].x ;
        double dy = linesEnd[0][0].y - linesEnd[0][1].y ;
        double dist = std::sqrt(dx * dx + dy * dy);
        if(dist<10){   //���߳��ȶ���10�����أ�����Ϊû�м�⵽����
            return ;
        }

        if(pIsLeft){ // ��ߵ�
            this->vertex = linesEnd[0][0].x<linesEnd[0][1].x?linesEnd[0][0]: linesEnd[0][1];
        }else{
            this->vertex = linesEnd[0][0].x>linesEnd[0][1].x?linesEnd[0][0]: linesEnd[0][1];
        }
        // ����ϵת��
        this->vertex.x = (this->vertex.x - 50)/10+min_X;
        this->vertex.y = (this->vertex.y - 50)/10+min_Y;

        // �����־λ
        this->vertexValidFlag = true;
        return ;
    }


    //------------ɸѡ������ֱ�ߣ��õ�ֱ�߹�ϵ,  ���ں����������϶����--------------
    if(linesEnd.size()<2){
        error="û�м�⵽����ֱ��";
        std::cout<<"û�м�⵽����ֱ��"<<std::endl;
        return;
    }

    while(linesEnd.size()>2){
        int delettIndex=0;
        float y=10000000;
        for(int i=0;i<linesEnd.size();i++){
            if((linesEnd[i][0].y+linesEnd[i][1].y)/2<y){
                y=(linesEnd[i][0].y+linesEnd[i][1].y)/2;
                delettIndex=i;
            }
        }
        linesEnd.erase(linesEnd.begin()+delettIndex);
    }

    cv::Mat fi;
    if(linesEnd.size() ==2){
        this->resultFlag=true;
    }
    std::vector<cv::Point2f> board, ref;

    //��ȡ���ߺͲο���
    board=linesEnd[0][0].x+linesEnd[0][1].x<linesEnd[1][0].x+linesEnd[1][1].x?linesEnd[0]:linesEnd[1];
    ref= linesEnd[0][0].x+linesEnd[0][1].x > linesEnd[1][0].x+linesEnd[1][1].x? linesEnd[0]:linesEnd[1]; //ȡx����


    if(!isleft){
        std::vector<cv::Point2f> tmp;
        tmp=board;
        board=ref;
        ref=tmp;
    }

    //չʾ���
    cv::Mat show_line;
    img.copyTo(show_line);
    line(show_line,board[0],board[1],cv::Scalar(255,0,0),4,cv::LINE_AA);
    line(show_line,ref[0],ref[1],cv::Scalar(0,255,0),4,cv::LINE_AA);
    this->show=show_line;

    float boardAngle,refAngle;
    boardAngle=getAngle(board[0],board[1]);
    refAngle=getAngle(ref[0],ref[1]);
    this->lidarAngle= refAngle-boardAngle; //����װ��Ϊ��׼���߲�Ϊ��ʱ����װ���ڰ�Ϊ������͹Ϊ��
    if(revAngle){
        this->lidarAngle=-this->lidarAngle;
    }

    //������Ӽ�� �� �߲�
    //���㷽����1. �ҳ���϶��Ӧ�Ķϵ㣬2.������  �߲refб�ʲ�����
    cv::Point2f board_point,ref_point;
    double min_x = 10000;
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<2;j++)
        {
            if(fabs(board[i].x-ref[j].x)<min_x)
            {
                min_x = fabs(board[i].x-ref[j].x);
                board_point = board[i];
                ref_point = ref[j];
            }
        }
    }
    //std::cout<<"----------------------------------"<<std::endl;
    //std::cout<<"x"<<board_point.x<<ref_point.x<<std::endl;
    //std::cout<<refAngle<<std::endl;

    lidarDist = (board_point.y-ref_point.y)*cos(refAngle/57.3)/10.0;
    gap = fabs(board_point.x-ref_point.x)/10.0;
    this->show=showImg(board,ref);
}

float LidarHelper::getAngle(cv::Point2f p1, cv::Point2f p2)
{
    float maxX=(p1.x>p2.x)?(p1.x):(p2.x);
    float maxY=(p1.y>p2.y)?(p1.y):(p2.y);
    float minX=(p1.x<p2.x)?(p1.x):(p2.x);
    float minY=(p1.y<p2.y)?(p1.y):(p2.y);

    float angle=atanf((p1.y-p2.y)/(p1.x-p2.x));
    angle=180*angle/M_PI;//��������
    return angle;
}

void LidarHelper::clear()
{
    resultFlag=0;
    lidarAngle=0;
    lidarDist=0;
    gap=0;
    vertexValidFlag = 0;
    vertex = cv::Point2f(0.0,0.0);
    cv::Mat newImg;
    show=newImg;
}

void LidarHelper::shrink()
{

    if(!show.empty()){
        if(show.channels()<3){//ת��ɫ
            cv::Mat color_show;
            cv::cvtColor(show, color_show, cv::COLOR_GRAY2BGR);
            show=color_show;
        }

        float scale1=400.0/float(show.rows);
        float scale2=400.0/float(show.cols);
        float scale=scale1>scale2?scale2:scale1;
        cv::Size dsize = cv::Size(show.cols*scale, show.rows*scale);
        cv::Mat shrink;
        cv:: resize(show, shrink, dsize, 0, 0, cv::INTER_AREA);
        show=shrink;
    }
}

float LidarHelper::getGap(std::vector<cv::Point2f> line1, std::vector<cv::Point2f> line2)
{
    float xlen1=abs(line1[0].x-line1[1].x);
    float xlen2=abs(line2[0].x-line2[1].x);
    float xmid1=(line1[0].x+line1[1].x)/2.0;
    float xmid2=(line2[0].x+line2[1].x)/2.0;
    float midxDist=abs(xmid1-xmid2);
    float gap=midxDist-(xlen1+xlen2)/2.0;
    return gap;
}

cv::Mat LidarHelper::showImg(std::vector<cv::Point2f> board, std::vector<cv::Point2f> ref)
{
    float scale=3;
    board[0]/=scale;
    board[1]/=scale;
    ref[0]/=scale;
    ref[1]/=scale;

    float maxNum=0,minNum=10000,minX=10000,minY=10000,maxY=0,maxX=0;
    minX=minX<board[0].x?minX:board[0].x;
    minX=minX<board[1].x?minX:board[1].x;
    minX=minX<ref[0].x?minX:ref[0].x;
    minX=minX<ref[1].x?minX:ref[1].x;
    minY=minY<board[0].y?minY:board[0].y;
    minY=minY<board[1].y?minY:board[1].y;
    minY=minY<ref[0].y?minY:ref[0].y;
    minY=minY<ref[1].y?minY:ref[1].y;
    ref[0].x-=minX;
    ref[0].y-=minY;
    ref[1].x-=minX;
    ref[1].y-=minY;
    board[0].x-=minX;
    board[0].y-=minY;
    board[1].x-=minX;
    board[1].y-=minY;


    maxX=maxX>board[0].x?maxX:board[0].x;
    maxX=maxX>board[1].x?maxX:board[1].x;
    maxX=maxX>ref[0].x?maxX:ref[0].x;
    maxX=maxX>ref[1].x?maxX:ref[1].x;

    maxY=maxY>board[0].y?maxY:board[0].y;
    maxY=maxY>board[1].y?maxY:board[1].y;
    maxY=maxY>ref[0].y?maxY:ref[0].y;
    maxY=maxY>ref[1].y?maxY:ref[1].y;

    maxNum=maxNum<board[0].x?board[0].x:maxNum;
    maxNum=maxNum<board[0].y?board[0].y:maxNum;
    maxNum=maxNum<board[1].x?board[1].x:maxNum;
    maxNum=maxNum<board[1].y?board[1].y:maxNum;
    maxNum=maxNum<ref[0].x?ref[0].x:maxNum;
    maxNum=maxNum<ref[0].y?ref[0].y:maxNum;
    maxNum=maxNum<ref[1].x?ref[1].x:maxNum;
    maxNum=maxNum<ref[1].y?ref[1].y:maxNum;

    minNum=minNum>board[0].x?board[0].x:minNum;
    minNum=minNum>board[0].y?board[0].y:minNum;
    minNum=minNum>board[1].x?board[1].x:minNum;
    minNum=minNum>board[1].y?board[1].y:minNum;
    minNum=minNum>ref[0].x?ref[0].x:minNum;
    minNum=minNum>ref[0].y?ref[0].y:minNum;
    minNum=minNum>ref[1].x?ref[1].x:minNum;
    minNum=minNum>ref[1].y?ref[1].y:minNum;

    int hight=2*maxNum;
//    hight=hight>2*maxNum?hight:2*maxNum;

    int bias=hight/3;
    ref[0].y+=bias;
    ref[1].y+=bias;
    board[0].y+=bias;
    board[1].y+=bias;

    //�õ��˵�
    int midx=(ref[0].x+ref[1].x+board[0].x+board[1].x)/4.0;
    cv::Point2d refmid,boardmid,refFar,boardFar;
    if(abs(ref[0].x-midx)<abs(ref[1].x-midx)){
        refmid=ref[0];
        refFar=ref[1];
    }else{
        refmid=ref[1];
        refFar=ref[0];
    }

    if(abs(board[0].x-midx)<abs(board[1].x-midx)){
        boardmid=board[0];
        boardFar=board[1];
    }else{
        boardmid=board[1];
        boardFar=board[0];
    }
    //�õ�ֱ��ƫ����
    float boardBias=(boardmid.y-boardFar.y)/(boardmid.x-boardFar.x)+0.0;
    boardBias=atan(boardBias);
    float refBias=(refmid.y-refFar.y)/(refmid.x-refFar.x)+0.0;
    refBias=atan(refBias);

    cv::Mat canvas(int(hight), int(maxX), CV_8UC3, cv::Scalar(150, 150, 150));
    cv::line(canvas,board[0],board[1],cv::Scalar(255,0,0),6, cv::LINE_AA);
    cv::line(canvas,boardmid, cv::Point2d(boardmid.x+hight*sin(boardBias),boardmid.y-hight*cos(boardBias)),
             cv::Scalar(255,0,0),6,cv::LINE_AA);
    line(canvas,ref[0],ref[1],cv::Scalar(0,255,0),6,cv::LINE_AA);
    line(canvas,refmid,cv::Point2d(refmid.x+hight*sin(refBias),refmid.y-hight*cos(refBias)),
         cv::Scalar(0,255,0),6,cv::LINE_AA);

    cv::Mat shrink;
    cv::Size dsize = cv::Size(200, 400);
    resize(canvas, shrink, dsize, 0, 0, cv::INTER_LINEAR);

    if(!Left){
        flip(shrink,shrink,1);
    }

    return shrink;
}


