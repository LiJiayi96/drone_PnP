#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/core/utility.hpp>
#include "opencv2/video.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>
#include <chrono>

using namespace cv;
using namespace std;



int main(int argc, char *argv[]) {



    VideoCapture cap(1);

    Mat frame;

    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    cout << cap.get(CAP_PROP_FPS) << endl;
    int frame_width=cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height=cap.get(CAP_PROP_FRAME_HEIGHT);
//    cap.set(CV_CAP_PROP)

    VideoWriter video1("/home/lijiayi/newdisk/Videotstfnal.avi",CV_FOURCC('X','V','I','D'),30, Size(frame_width,frame_height),true);

    auto startT= std::chrono::high_resolution_clock::now();

    auto ttt0= std::chrono::high_resolution_clock::now();
//    std::chrono::high_resolution_clock::time_point ttt;
    chrono::duration<double> period;
    auto ttt= ttt0;
    while(cap.read(frame)){
//        cap >> frame;
        ttt0 =ttt;
        ttt=chrono::high_resolution_clock::now();
        period=ttt-ttt0;
        double t=period.count();
        cout << t << endl;
        imshow("camera",frame);
        video1 << frame;
//        video1.write(frame);
        if (waitKey(1) >= 0)
        {
            ttt=chrono::high_resolution_clock::now();
            period=ttt-startT;
            double t=period.count();
            cout << t<<endl;
            break;
        }
    }

    return 0;
}