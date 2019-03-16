#include <iostream>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

const int g_nMaxAlohaValue = 100;
int g_nAlphaValueSlider;
double g_dAlphaValue;
double g_dBetaValue;

Mat g_srcImg1;
Mat g_srcImg2;
Mat g_dstImg;

void on_Trackbar(int, void *) {
    g_dAlphaValue = (double) g_nAlphaValueSlider / g_nMaxAlohaValue;
    g_dBetaValue = 1.0 - g_dAlphaValue;

    addWeighted(g_srcImg1, g_dAlphaValue, g_srcImg2, g_dBetaValue, 0.0, g_dstImg);
    imshow("Result", g_dstImg);

}


int main() {
    g_srcImg1 = imread("/home/lijiayi/图片/1.jpg");
    g_srcImg2 = imread("/home/lijiayi/图片/2.jpg");
    if (!g_srcImg1.data)return -1;
    g_nAlphaValueSlider = 70;
    namedWindow("ResultWindow", 1);

    createTrackbar("toumingdu", "ResultWindow", &g_nAlphaValueSlider, g_nMaxAlohaValue, on_Trackbar);

    Mat M(2, 2, CV_8UC3, Scalar(1, 100, 255));
    cout << M << endl;

    waitKey(0);

    return 0;
}