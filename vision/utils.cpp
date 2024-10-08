
#include<string>
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include<ctime>
#include<chrono>
#include "utils.h"


using namespace std;


/*  图像处理相关方法  */
cv::Mat padding_hollow(cv::Mat mask, int ksize ) {

    cv::Mat kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));
    // 进行闭操作
    cv::Mat closed_image;
    morphologyEx(mask, closed_image, cv::MORPH_CLOSE, kernel);
    return closed_image;

}

std::vector<std::vector<float>> get_line_by_lsd(cv::Mat bin_img) {


    //基于LSD算法获取线检测点
    cv::Ptr<cv::LineSegmentDetector> detector = createLineSegmentDetector(cv::LSD_REFINE_STD);
//    LSD_REFINE_STD
    std::vector<cv::Vec4f> lines;
    std::vector<std::vector<float>> line_res;
    detector->detect(bin_img, lines);


    for (size_t i = 0; i < lines.size(); i++) {

        cv::Vec4f line = {
                round(lines[i][0]),
                round(lines[i][1]),
                round(lines[i][2]),
                round(lines[i][3])
        };
        double dist = calculatePointsDistance(cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]));

        // 过滤掉距离过小的线段
        if (dist <512*0.15) { continue;}   // 不过滤小线段的话，会导致预测的结果存在偏差，因为小线段拟合的直线可能不准确

//        //直线拟合
//        std::vector<cv::Point2f> points = {
//                cv::Point2f(line[0], line[1]),
//                cv::Point2f(line[2], line[3])
//        };

//        vector<float> line_fited = fiting_line(points);
//        if(line_fited.size()>0){
//            line_res.push_back(line_fited);
//        }
        std::vector<float> line_fited = {line[0], line[1], line[2], line[3]};
        line_res.push_back(line_fited);
    }

    return line_res;
}

double computePointToLineDistance(std::vector<float> line_1, std::vector<float> line_2) {

    double Slope_1 =line_1[4];
    double C1 = line_1[1];

    double Slope_2 =line_2[4];
    double C2 = line_2[1];

    // 直线上的某点
//    double X1 = line_1[0];
//    double Y1 = line_1[1];
    //直线上的中点
    double X1 = (line_1[0]+line_1[2])/2;
    double Y1 = (line_1[1]+line_1[3])/2;
    // 点斜式转为标准式
    double A2 = -Slope_2;
    double B2 =1;
    C2 = -C2;
    // 函数用于计算点到直线的垂直距离
    return perpendicularDistance(A2, B2, C2, X1, Y1);
}
double perpendicularDistance(double A, double B, double C, double x1, double y1) {
    return std::abs(A * x1 + B * y1 + C) / std::sqrt(A * A + B * B);
}


double calculateDistanceBetweenParallelLines(double slope, double intercept1, double intercept2) {
    double denominator = std::sqrt(1 + slope * slope);
    return std::abs(intercept2 - intercept1) / denominator;
}

double calculatePointsDistance(const cv::Point2f A, const cv::Point2f B) {
    /*
      计算两个点之间的欧式距离
    */

    return sqrt(pow(A.x - B.x, 2) + pow(A.y - B.y, 2));
}

vector<float> fiting_line(vector<cv::Point2f> points) {
	/**
	 * @brief 根据点拟合成一条直线
	 *
	 * 根据给定的点集，使用拟合算法拟合成一条直线，并返回起始四个点的坐标信息。
	 *
	 * @param points 点的集合
	 *
	 * @return 拟合直线的起始四个点坐标
	 */

    vector<float> res;

    double param = 0; //自动最佳参数
    double reps = 0.01;// 坐标原点与直线之间的距离
    double aeps = 0.01; // 角度精度

    cv::Vec4f lines;
    cv::fitLine(points, lines, cv::DIST_L1, param, reps, aeps);

    float k = lines[1] / lines[0];
    float lefty = (-lines[2] * k) + lines[3];
    float righty = ((512 - 1 - lines[2])*k) + lines[3];

    res.push_back(0.0);
    res.push_back(lefty);
    res.push_back(512.0);
    res.push_back(righty);
    res.push_back(k);
    return res;
}

void sort_lines(std::vector<std::vector<float>> &lines, bool ascending) {
    if(lines.empty()){
        return ;
    }
    std::sort(lines.begin(), lines.end(), [ascending](const vector<float> &a, const vector<float> &b){
        if(ascending){
            float min_a_y  = std::min(a[1], a[3]);
            float min_b_y  = std::min(b[1], b[3]);
            return min_a_y < min_b_y;
        }else{
            // 当需要按降序比较时，比较两条线段在y轴上的最大值
            float max_a_y = std::max(a[1], a[3]);
            float max_b_y = std::max(b[1], b[3]);
            return max_a_y > max_b_y;
        }
    });
}

bool lineIsOverlapping( std::vector<float> a, std::vector<float> b, double tolerance = 30) {
    // 这里使用简单的逻辑来判断线段是否重合
    // 你可以根据需要添加更复杂的逻辑，比如检查线段是否足够接近
    return (std::abs(a[0] - b[0]) < tolerance && std::abs(a[1] - b[1]) < tolerance &&
            std::abs(a[2] - b[2]) < tolerance && std::abs(a[3] - b[3]) < tolerance);
}


std::vector<std::vector<float>> linesAggregation(std::vector<std::vector<float>> lines){
    // 合并重合的直线

    int num_lines = lines.size();
    std::vector<bool> isVisited(num_lines);
    std::fill_n(isVisited.begin(), num_lines, false);
    std::vector<std::vector<float>> lineRes;

    for (int i = 0; i < num_lines; i++) {
        if(isVisited[i]){
            continue;
        }else{
            isVisited[i] = true;
        }
        int index = -1;
        for(int j = i+1; j < num_lines; j++){
            if(isVisited[j]){
                continue;
            }
            if(lineIsOverlapping(lines[i], lines[j], 30)){ // 30为阈值，表示两个线段重合的阈值
                isVisited[j] = true;
                index = j;
            }
        }

        if(index != -1){
            std::vector<float> line = {
                    (lines[i][0] + lines[index][0]) / 2,
                    (lines[i][1] + lines[index][1]) / 2,
                    (lines[i][2] + lines[index][2]) / 2,
                    (lines[i][3] + lines[index][3]) / 2,
                    (lines[i][4] + lines[index][4]) / 2
            };

            lineRes.push_back(line);
        }else{
            lineRes.push_back(lines[i]);
        }
    }
    return lineRes;
}


double calculateDistance(const vector<float> A, const vector<float> B) {
	/**
	 * @brief 计算两个向量之间的余弦相似度
	 *
	 * 使用余弦相似度公式计算两个向量 A 和 B 之间的距离。
	 *
	 * @param A 向量 A
	 * @param B 向量 B
	 *
	 * @return 两个向量之间的余弦相似度
	 */

    double sum_A = 0, sum_B = 0;
    float len = static_cast<float>(A.size());
    for (int i = 0; i < A.size(); i++) {
        sum_A += A[i];
        sum_B += B[i];
    }
    sum_A = sum_A / len;
    sum_B = sum_B / len;
    double res = sum_A - sum_B;
    res = res < 0 ? res * -1 : res;
    return res;
}

bool check_is_boundary(cv::Mat img, cv::Vec4f line) {

	/**
	 * @brief 粗略评估检查是否为板线边界
	 *
	 * 使用给定的图像和直线，评估该直线是否为板线的边界。
	 *
	 * @param img 灰度待检测图像
	 * @param line 可能的板线边界
	 *
	 * @return 如果直线是板线的边界，则返回 true；否则返回 false
	 */
    
    cv::Mat img_32F;
    bool flag = true;
    float thr = 180;  //相似性阈值
    img.convertTo(img_32F, CV_32F);
    int x1 = line[0];
    int y1 = line[1];
    int x2 = line[2];
    int y2 = line[3];

    int offset = 30;
    // 获取中间坐标
    int base_x = min(x1, x2) + abs(x1 - x2) / 2;
    int base_y = min(y1, y2) + abs(y1 - y2) / 2;


    int base_up_y = (base_y - offset) > 0 ? (base_y - offset) : 1;
    int base_down_y = (base_y + offset) < img.rows - 1 ? (base_y + offset) : img.rows - 2;

    // 分别获取上下点的四邻域，并计算相似度；
    vector<float> block_1 = {
        img_32F.at<float>(base_up_y,base_x),//中心
        img_32F.at<float>(base_up_y - 1,base_x),//中心上
        img_32F.at<float>(base_up_y + 1,base_x),//中心下

       img_32F.at<float>(base_up_y,base_x - 1),//中心左
       img_32F.at<float>(base_up_y,base_x + 1),//中心右

       img_32F.at<float>(base_up_y - 1,base_x - 1),//左上
       img_32F.at<float>(base_up_y + 1,base_x - 1),//左下

       img_32F.at<float>(base_up_y - 1,base_x + 1),//右上
       img_32F.at<float>(base_up_y + 1,base_x + 1),//右下
    };


    vector<float> block_2 = {
       img_32F.at<float>(base_down_y,base_x),  //中心
       img_32F.at<float>(base_down_y - 1,base_x),//中心上
       img_32F.at<float>(base_down_y + 1,base_x),//中心下

       img_32F.at<float>(base_down_y,base_x - 1),//中心左
       img_32F.at<float>(base_down_y,base_x + 1),//中心右

       img_32F.at<float>(base_down_y - 1,base_x - 1),//左上
       img_32F.at<float>(base_down_y + 1,base_x - 1),//左下

       img_32F.at<float>(base_down_y - 1,base_x + 1),//右上
       img_32F.at<float>(base_down_y + 1,base_x + 1),//右下
    };

    float score = calculateDistance(block_1, block_2);
    if (score < thr) {
        flag = false;
    }
    return flag;
}

void draw_line(vector<cv::Vec4f> lines, cv::Mat& img) {
    /*
       绘制直线到图像上，便于可视化
    */
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4f line = lines[i];
        line[0] = round(line[0]);
        line[1] = round(line[1]);
        line[2] = round(line[2]);
        line[3] = round(line[3]);
        cv::line(img, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 2);
    }
}

void draw_line(vector<vector<float>> lines, cv::Mat& img) {
    /*
       绘制直线
    */

    for (const auto& row : lines) {
        int start_x = static_cast<int>(row[0]);
        int start_y = static_cast<int>(row[1]);
        int end_x = static_cast<int>(row[2]);
        int end_y = static_cast<int>(row[3]);

        cv::Point start(start_x, start_y);
        cv::Point end(end_x, end_y);
        cv::line(img, start, end, cv::Scalar(0, 0, 255), 2);
    }

}

cv::Mat rotateImage(const cv::Mat src) {
	// 获取图像尺寸
	int rows = src.rows;
	int cols = src.cols;

	// 创建一个新的Mat对象来存储旋转后的图像
	cv::Mat rotatedImage(cols, rows, src.type());
	// 使用cv::rotate函数旋转图像
	// 注意OpenCV的rotate函数默认是顺时针旋转，所以我们需要使用ROTATE_90_COUNTERCLOCKWISE标志
	cv::rotate(src, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
	return rotatedImage;
}

// 函数用于获取当前时间并转换为字符串
std::string getCurrentTimeAsString() {
    // 获取当前时间
    //auto now = std::chrono::system_clock::now();
    //// 转换为time_t以使用ctime库
    //std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    //// 使用ctime库将time_t转换为本地时间
    //std::tm* local_time = std::localtime(&now_time_t);

    //// 格式化时间字符串
    //std::ostringstream time_str;
    //time_str << std::put_time(local_time, "%Y%m%d%H%M%S");
    return "";
}


double compareHistograms(const cv::Mat& hist1, const cv::Mat& hist2) {
    // 使用OpenCV的compareHist函数，这里选择CORREL方法（相关性）
    // 其他方法包括：CV_COMP_CHISQR, CV_COMP_CHISQR_ALT, CV_COMP_INTERSECT, CV_COMP_BHATTACHARYYA
    double similarity = cv::compareHist(hist1, hist2, cv::HISTCMP_CORREL);
    return similarity;
}

double calculateHistogramSimilarity(const cv::Mat& img1, const cv::Mat& img2) {

    if(img1.size()!=img2.size()){
        std::cerr << "Error: Images must be of the same size!" << std::endl;
        return -1;
    }

    // 定义直方图的参数
    const int channels[] = {0, 1, 2}; // B, G, R
    int histSize[] = {10, 10, 10}; // bin的数量
    float ranges[] = {0, 256}; // 像素值范围
    const float* histRange[] = {ranges};

    cv::Mat hist1, hist2;
    int histSizeOneD = histSize[0] * histSize[1] * histSize[2]; // 合并为一个维度
    int channelsUsed[] = {0}; // 使用所有通道合并为一个直方图

    // 计算第一个图像的直方图
    cv::calcHist(&img1, 1, channelsUsed, cv::Mat(), hist1, 1, &histSizeOneD, &histRange[0], true, false);
    cv::normalize(hist1, hist1, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    // 计算第二个图像的直方图
    cv::calcHist(&img2, 1, channelsUsed, cv::Mat(), hist2, 1, &histSizeOneD, &histRange[0], true, false);
    cv::normalize(hist2, hist2, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    // 比较两个直方图
    double similarity = compareHistograms(hist1, hist2);
    return similarity;
}

bool check_is_border_line(cv::Mat img, cv::Vec4f line, float thr) {
    bool flag = false;
    int offset = 30; // 获取上下点的四邻域，并计算相似度；

    // 获取直线坐标
    int x1 = line[0];
    int y1 = line[1];
    int x2 = line[2];
    int y2 = line[3];

    // 获取中间坐标
    int base_x = min(x1, x2) + abs(x1 - x2) / 2;
    int base_y = min(y1, y2) + abs(y1 - y2) / 2;

    int  base_up_y = base_y - offset;
    int base_down_y  = base_y + offset;

    if(base_up_y < 0 || (base_down_y>img.rows-1)){
        return false;  // 超出边界
    }

    if((base_x+offset-10)>512){
        return false;
    }



    cv::Rect roi_img_1(base_x, base_up_y, offset-10,offset-10);
    cv::Rect roi_img_2(base_x, base_y+10, offset-10,offset-10);

    cv::Mat img_cropped_1 = img(roi_img_1);
    cv::Mat img_cropped_2 = img(roi_img_2);

    img_cropped_1.convertTo(img_cropped_1, CV_32F);
    img_cropped_2.convertTo(img_cropped_2, CV_32F);

    cv::Mat matDifference;
    cv::subtract(img_cropped_1, img_cropped_2, matDifference);
    cv::Scalar meanScalar = cv::mean(matDifference);
    double similarty = abs(static_cast<double>(meanScalar[0]))/255.0;

//    double similarty = calculateHistogramSimilarity(img_cropped_1, img_cropped_2);

//    similarty = (similarty+1)/2;
//    std::cout << "similarty: " << similarty << std::endl;
//    cv::circle(img, cv::Point(base_x, base_y), 5, cv::Scalar(255, 0, 0), -1);
//    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255,255),2);
//    cv::rectangle(img, roi_img_1, cv::Scalar(0, 0, 255), 2);
//    cv::rectangle(img, roi_img_2, cv::Scalar(0, 0, 255), 2);
//    cv::imshow("img", img);
//    cv::waitKey(0);
    if(similarty > 0.6){
        flag = true;
    }
    return flag;
}

bool check_is_ink_line(cv::Mat img, cv::Vec4f line) {
    bool flag = false;
    float thr = 0.7;
    int offset = 30; // 获取上下点的四邻域，并计算相似度；

    // 获取直线坐标
    int x1 = line[0];
    int y1 = line[1];
    int x2 = line[2];
    int y2 = line[3];

    // 获取中间坐标
    int base_x = min(x1, x2) + abs(x1 - x2) / 2;
    int base_y = min(y1, y2) + abs(y1 - y2) / 2;

    int base_up_y = (base_y - offset) > 0 ? (base_y - offset) : 1;
    int base_down_y = (base_y + offset) < img.rows - 1 ? (base_y + offset) : img.rows - 2;

    if(base_x+offset>img.cols-1 || base_up_y+offset>img.rows-1 || base_down_y+offset>img.rows-1){
//        std::cout << "base_x: " << base_x << ", base_up_y: " << base_up_y << ", base_down_y: " << base_down_y << std::endl;
        return false;
    }
    cv::Rect roi_img_1(base_x, base_up_y, offset,offset);
    cv::Rect roi_img_2(base_x, base_down_y, offset,offset);

    cv::Mat img_cropped_1 = img(roi_img_1);
    cv::Mat img_cropped_2 = img(roi_img_2);

    double similarty = calculateHistogramSimilarity(img_cropped_1, img_cropped_2);
    similarty = (similarty+1)/2;
    if(similarty > thr){
        flag = true;
    }
    return flag;
}

std::string getCurrentTimestampString() {
    // 获取当前时间
    std::time_t now = std::time(nullptr);

    // 转换为本地时间
    std::tm* local_time = std::localtime(&now);

    // 创建一个输出字符串流
    std::ostringstream oss;

    // 使用put_time来格式化时间
    oss << std::put_time(local_time, "%Y-%m-%d-%H-%M-%S");

    // 返回字符串
    return oss.str();
}


void imageOpenedOrClosed(cv::Mat &src, int kernelSize, bool isOpened) {

    cv::GaussianBlur(src, src, cv::Size(kernelSize, kernelSize), 0);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    if(isOpened){
        cv::morphologyEx(src, src, cv::MORPH_OPEN, kernel);
    }else{
        cv::morphologyEx(src, src, cv::MORPH_CLOSE, kernel);
    }
}
