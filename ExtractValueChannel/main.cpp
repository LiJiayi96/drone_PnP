#include <iostream>
#include <fstream>

#include <opencv2/core/utility.hpp>
#include "opencv2/video.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "cmath"

using namespace cv;
using namespace std;

class myRotatedRect : public RotatedRect {
public:
    static bool isSimilar(RotatedRect myRotatedRect1,RotatedRect myRotatedRect2) {
        if (pow(myRotatedRect2.center.x - myRotatedRect1.center.x, 2) + pow(myRotatedRect2.center.y - myRotatedRect1.center.y, 2) >
            (20 * 20))
            return false;
        if(myRotatedRect2.size.area()/myRotatedRect1.size.area()>1.4 || myRotatedRect2.size.area()/myRotatedRect1.size.area()<.7)
            return false;
        return true;
    }
};


int frame_width=1920;
int frame_height=1080;

VideoWriter contoursVideo("contoursVideo.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),false);
VideoWriter resultVideo("resultVideo.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);

void imgProcess(Mat img, Mat &rectImg, vector<RotatedRect> &arrayRect) {
    namedWindow("Source", WINDOW_NORMAL);
    imshow("Source", img);
    Mat grayImg, hsvImg, binImg;
//    cvtColor(img, grayImg, CV_BGR2GRAY);
    grayImg= Mat::zeros(img.size(), CV_8UC1);
    cvtColor(img, hsvImg, CV_BGR2HSV);



    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            const int hi = i * img.cols * 3 + j * 3,
                    gi = i * img.cols + j;
            grayImg.data[gi] = hsvImg.data[hi + 2];
        }
    }
////通道图像
//    namedWindow("V-channel", WINDOW_NORMAL);
//    imshow("V-channel", grayImg);
    threshold(grayImg, binImg, 245, 255, THRESH_BINARY);
////二值化图像
//    namedWindow("Binary", WINDOW_NORMAL);
//    imshow("Binary", binImg);

//    imwrite("/home/lijiayi/CLionProjects/ExtractValueChannel/5bin.jpg",binImg);

//region 膨胀
    Mat dilatedImg;
    int structElementSize = 3;
    Mat structElement = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1),
                                              Point(structElementSize, structElementSize));
    dilate(binImg, dilatedImg, structElement);
    namedWindow("Dilated", WINDOW_NORMAL);
    imshow("Dilated", dilatedImg);
//endregion

//region 寻找轮廓
    vector<vector<Point>> contours;

    vector<Vec4i> hierarchy;
    findContours(dilatedImg, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

    Mat contoursImg = Mat::zeros(img.size(), CV_8UC1);
    rectImg = img.clone();

    drawContours(contoursImg, contours, -1, Scalar(255), 3, 8, hierarchy);

//endregion

    vector<vector<Point>> approxShape;
    vector<RotatedRect> contoursRect;

    for (auto itContour = contours.begin(); itContour != contours.end(); itContour++) {
//        //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
//        for(int j=0;j<contours[i].size();j++)
//        {
//            //绘制出contours向量内所有的像素点
//            Point P=Point(contours[i][j].x,contours[i][j].y);
//            Contours.at<uchar>(P)=255;
//        }
//
//        //输出hierarchy向量内容
//        char ch[256];
//        sprintf(ch,"%d",i);
//        string str=ch;
//        cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;

        if (contourArea(*itContour) < 1000) {
//            contours.erase(itContour);
            continue;
        }
        RotatedRect t_rect;
        t_rect = minAreaRect(*itContour);
        bool isnew=true;
        for(auto it=arrayRect.begin();it<arrayRect.end();it++)
        {
            if(myRotatedRect::isSimilar(t_rect,*it))
            {
                *it=t_rect;
                isnew=false;
            }
        }
        if(isnew)
            arrayRect.push_back(t_rect);
//        contoursRect.push_back(t_rect);


////region 多边形近似
//        vector<Point> t_contours;
//        approxPolyDP(*itContour, t_contours, 10, false);
//        approxShape.push_back(t_contours);
////endregion
        //绘制轮廓

//        namedWindow("Contours Image"+to_string(i), WINDOW_NORMAL);
//        imshow("Contours Image"+to_string(i),contoursImg); //轮廓

    }
//绘制轮廓与矩形
    for(auto it=arrayRect.begin();it<arrayRect.end();it++)
    {
        Point2f vertices[4];
        (*it).points(vertices);
        for (int i = 0; i < 4; i++)
            line(rectImg, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 4);
    }
    namedWindow("Contours Image", WINDOW_NORMAL);
    imshow("Contours Image", contoursImg); //轮廓
    contoursVideo.write(contoursImg);
    resultVideo.write(rectImg);
//
//    namedWindow("Rect Image", WINDOW_NORMAL);
//    imshow("Rect Image", rectImg); //轮廓

////region 多边形近似输出
//    Mat approxImg = img.clone();
//    for (int i = 0; i < approxShape.size(); i++) {
//        for (int j = 0; j < approxShape[i].size() - 1; j++) {
//            line(approxImg, approxShape[i][j], approxShape[i][j + 1], Scalar(0, 255, 0), 5);
//        }
//    }
//    namedWindow("Approx Image", WINDOW_NORMAL);
//    imshow("Approx Image", approxImg);
////endregion


////region 霍夫线变换
//
//    vector<Vec4i> lines;
//    double rho = 10, theta = CV_PI * 10 / 180, minLineLength = 100, maxLineGap = 20;
//    int houghThreshold = 200;
//    HoughLinesP(contoursImg, lines, rho, theta, houghThreshold, minLineLength, maxLineGap);
//
//    Mat linesImg = img.clone();
//    for (size_t i = 0; i < lines.size(); i++) {
//        Vec4i l = lines[i];
//        line(linesImg, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 3);
//
////        namedWindow("Lines Image"+to_string(i), WINDOW_NORMAL);
////        imshow("Lines Image" + to_string(i), linesImg); //轮廓
//
//    }
//
//    namedWindow("Lines Image", WINDOW_NORMAL);
//    imshow("Lines Image", linesImg); //轮廓
//// endregion



//    namedWindow("Point of Contours", WINDOW_NORMAL);
//    imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集

}

int main() {
    VideoCapture cap("/home/lijiayi/CLionProjects/VideoFiles/GOPR1.MP4");
//    img = imread("/home/lijiayi/CLionProjects/VideoFiles/GOPR2.JPG");

    Mat frame, outImg;
    vector<RotatedRect> arrayRect;
    namedWindow("Processed", WINDOW_FREERATIO);
    for (;;) {
        if (cap.read(frame)) {
            imgProcess(frame, outImg, arrayRect);
            imshow("Processed", outImg);
        }
        if (waitKey(1) >= 0) break;
    }
    return 0;
}