#include <iostream>
#include <fstream>

#include <opencv2/core/utility.hpp>
#include "opencv2/video.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "cmath"

using namespace cv;
using namespace std;


void imgProcess(Mat img, Mat &rectImg, vector<RotatedRect> &arrayRect) {
    namedWindow("Source", WINDOW_NORMAL);
    imshow("Source", img);
    Mat grayImg, hsvImg, binImg;
    cvtColor(img, grayImg, CV_BGR2GRAY);
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

        if (contourArea(*itContour) < 1000) {
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
//
//    namedWindow("Rect Image", WINDOW_NORMAL);
//    imshow("Rect Image", rectImg); //轮廓


}

Point2f point;
bool addRemovePt = true;

int main() {

    VideoCapture cap;
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);  //the situation we want to terminate, maxcount=20, epsilon=0.03
    Size subPixWinSize(10,10), winSize(31,31);

    const int MAX_COUNT = 500;
    bool needToInit = true;
    bool nightMode = false;

//    cv::CommandLineParser parser(argc, argv, "{@input|0|}");
//    string input = parser.get<string>("@input");
    string input("/home/lijiayi/CLionProjects/VideoFiles/GOPR1.MP4");

    if( input.size() == 1 && isdigit(input[0]) )
        cap.open(input[0] - '0');
    else
        cap.open(input);

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

    namedWindow( "Processed", WINDOW_NORMAL );
//    setMouseCallback( "Processed", onMouse, 0 );

    Mat gray, prevGray, image, frame;
    vector<Point2f> points[2];  //prevPoints vector and nextPoints vector

    for(;;)
    {
        cap >> frame;
        if( frame.empty() ) //break;
            continue;

//        float scale=0.5;
//        resize(frame,frame,Size(frame.cols*scale,frame.rows*scale));

        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);
//region
        Mat grayImg, hsvImg, binImg;
        grayImg= Mat::zeros(frame.size(), CV_8UC1);
        cvtColor(frame, hsvImg, CV_BGR2HSV);


        for (int i = 0; i < frame.rows; i++) {
            for (int j = 0; j < frame.cols; j++) {
                const int hi = i * frame.cols * 3 + j * 3,
                        gi = i * frame.cols + j;
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

        Mat contoursImg = Mat::zeros(frame.size(), CV_8UC1);
        rectImg = frame.clone();

        drawContours(contoursImg, contours, -1, Scalar(255), 3, 8, hierarchy);

//endregion

        vector<vector<Point>> approxShape;
        vector<RotatedRect> contoursRect;

        for (auto itContour = contours.begin(); itContour != contours.end(); itContour++) {

            if (contourArea(*itContour) < 1000) {
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

     //endregion

        if( nightMode )
            image = Scalar::all(0);

        if( needToInit ) //false by default
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
            addRemovePt = false;
        }
        else if( !points[0].empty() ) //if there're points
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                if( addRemovePt )
                {
                    if( norm(point - points[1][i]) <= 5 )
                    {
                        addRemovePt = false;
                        continue;
                    }
                }

                if( !status[i] )
                    continue;

                points[1][k++] = points[1][i];
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
            }
            points[1].resize(k);
        }

        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            tmp.push_back(point);
            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            points[1].push_back(tmp[0]);
            addRemovePt = false;
        }

        needToInit = false;
        imshow("LK Demo", image);

        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch( c )
        {
            case 'r':
                needToInit = true;
                break;
            case 'c':
                points[0].clear();
                points[1].clear();
                break;
            case 'n':
                nightMode = !nightMode;
                break;
        }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }

    return 0;
}