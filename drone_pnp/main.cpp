#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/video/background_segm.hpp>
#include "opencv2/bgsegm.hpp"
#include <vector>
#include <opencv2/viz.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#define PENGZHANG 1
#define THETA_X (30*3.141592653589/180)
#define THETA_Y 0
#define THETA_Z 3.141592653589

#define CAPINDEX 1

using namespace cv;
using namespace std;

//const char* params
//        = "{ help h         |                   | Print usage }"
//          "{ input          | /media/lijiayi/OS/Users/69067/Pictures/Camera Roll/WIN_20190121_20_19_00_Pro.mp4 | Path to a video or a sequence of image }"
//          "{ algo           | 1MOG2              | Background subtraction method (KNN, MOG2) }";
class myImgPoint : public Point2f
{
public:
    float angle_with_center;
    float length_to_next_point;
    myImgPoint(float x,float y):  Point2f(x,y){ }
    bool operator < (const myImgPoint& myImgPoint1) const
    {
     return angle_with_center<myImgPoint1.angle_with_center;
    }
    bool operator > (const myImgPoint& myImgPoint1) const
    {
     return angle_with_center>myImgPoint1.angle_with_center;
    }
};

float dist_btn_pts(Point2f p1,Point2f p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) * 1.0);
}


int main(int argc, char* argv[]) {
//    CommandLineParser parser(argc, argv, params);
//    parser.about("This program shows how to use d provided by "
//                 " OpenCV. You can process both videos and images.\n");
//    if (parser.has("help")) {
//        //print help information
//        parser.printMessage();
//    }
    String algo="MOG";


    //相机参数
    Mat cameraMatrix;
    cameraMatrix = Mat::zeros(3, 3, CV_64F);
    const vector<double> distCoeffs{0.03819613038058793, -0.06112508916119349, -5.515361356058103e-05,
                                    0.003470678820019871, 0};

    cameraMatrix.at<double>(0, 0) = 411.1663471332097;
    cameraMatrix.at<double>(0, 2) = 331.9546551659722;
    cameraMatrix.at<double>(1, 1) = 411.4132376251969;
    cameraMatrix.at<double>(1, 2) = 268.2985302141446;
    cameraMatrix.at<double>(2, 2) = 1;

    //! [create]
    //create Background Subtractor objects
    Ptr<BackgroundSubtractor> pBackSub;
    if (algo == "MOG2")
        pBackSub = createBackgroundSubtractorMOG2();
    else if(algo == "KNN")
        pBackSub = createBackgroundSubtractorKNN();
    else if(algo == "MOG")
        pBackSub = bgsegm::createBackgroundSubtractorMOG(); //MOG approach
    //! [create]

    //! [capture]
    VideoCapture capture(CAPINDEX);
    capture.set(CAP_PROP_AUTO_EXPOSURE,0.25);
    capture.set(CAP_PROP_EXPOSURE, 0.00001);
    if (!capture.isOpened()) {
        //error in opening the video input
        cerr << "Unable to open: " << CAPINDEX << endl;
        return 0;
    }
    //! [capture]

    //region 3D virtualization
    viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
    viz::WCube cube_widget1(Point3f(-0.0663, -0.03966, -0.01), Point3f(0.0663, 0, 0.01), false, viz::Color::blue());
    viz::WCube cube_widget2(Point3f(-0.03135, 0, -0.01), Point3f(0.03135, 0.06132, 0.01), false, viz::Color::blue());
    myWindow.showWidget("Cube Widget1", cube_widget1);
    myWindow.showWidget("Cube Widget2", cube_widget2);
    viz::WLine line_widget(Point3d(0, 0, 3), Point3d(0, 0, 0), viz::Color::red());
    myWindow.showWidget("line", line_widget);
    line_widget.setRenderingProperty(viz::LINE_WIDTH, 1);
    myWindow.spinOnce(1, true);

//endregion

    //region camera position transform
//    Vec3d rvec1 = Vec3d(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0));
//    Vec3d tvec1 = Vec3d(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0));
//    Affine3f pose(rvec1,tvec1);
//    cout << pose.matrix << endl;
    //endregion

    Mat frame, fgMask, frame_masked, binImg, hsvImg, img, valueImg, result;
    img = Mat::zeros(480, 640, CV_8UC3);
    valueImg = Mat::zeros(480, 640, CV_8UC1);
    result = Mat::zeros(1000, 1000, CV_8UC1);

    ofstream myfile;
    myfile.open("/home/lijiayi/newdisk/cameraresult.csv");
    int n = 0;

    while (true) {
        capture >> frame;
        if (frame.empty())
            break;
        undistort(frame, img, cameraMatrix, distCoeffs);
        imshow("undistorted", img);

        cvtColor(img, hsvImg, CV_BGR2HSV);

        for (int i = 0; i < img.rows; i++) {
            for (int j = 0; j < img.cols; j++) {
                const int hi = i * img.cols * 3 + j * 3,
                        gi = i * img.cols + j;
                valueImg.data[gi] = hsvImg.data[hi + 2];
            }
        }

        //! [apply]
        //update the background model
        pBackSub->apply(img, fgMask);
        //! [apply]
        frame_masked = Mat::zeros(frame.size(), frame.type());
        binImg = frame_masked.clone();
        valueImg.copyTo(frame_masked, fgMask);
        threshold(frame_masked, binImg, 200, 255, THRESH_BINARY);



        //! [show]
        //show the current frame and the fg masks
//        imshow("Frame", frame);
//        imshow("FG Mask", fgMask);
        imshow("frame_masked", frame_masked);
//        imshow("bin",binImg);
        //! [show]

        //region 膨胀
        Mat dilatedImg;
        if (PENGZHANG) {
            int structElementSize = 1;
            Mat structElement = getStructuringElement(MORPH_ELLIPSE,
                                                      Size(2 * structElementSize + 1, 2 * structElementSize + 1),
                                                      Point(structElementSize, structElementSize));
            dilate(binImg, dilatedImg, structElement);
        } else {
            dilatedImg = binImg.clone();
        }

//        imshow("Dilated", dilatedImg);
        //endregion

        //region 寻找轮廓
        vector<vector<Point>> contours, contours_org;

        vector<Vec4i> hierarchy;
        findContours(dilatedImg, contours_org, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

        //region find max_Area
        double maxArea;
        vector<Point> temp;
        for (auto i = contours_org.begin(); i != contours_org.end(); i++) {
            maxArea = contourArea(*i);
            for (auto j = i + 1; j != contours_org.end(); j++) {
                if (contourArea(*j) > maxArea) {
                    maxArea = contourArea(*j);
                    temp = *i;
                    (*i) = (*j);
                    (*j) = temp;
                }
            }
        }
        if (contours_org.size() >= 4)
            for (int i = 0; i < 4; i++) contours.push_back(contours_org[i]);
        //endregion


        Mat contoursImg = Mat::zeros(img.size(), CV_8UC1);
//          rectImg = img.clone();

//            drawContours(contoursImg, contours, -1, Scalar(255), 3, 8, hierarchy);

//endregion



        vector<myImgPoint> imgPts;
        vector<Point2f>  imgPts2;


        for (auto itContour = contours.begin(); itContour != contours.end(); itContour++) {
//            if(contourArea(*itContour)<40)
//            {
//                continue;
//            }

            Moments m;
            m = moments(*itContour, true);
            myImgPoint p = myImgPoint((m.m10 / m.m00), (m.m01 / m.m00));
            imgPts.push_back(p);

            for (auto itContourPt = (*itContour).begin(); itContourPt != (*itContour).end(); itContourPt++) {
                contoursImg.at<uchar>(Point((*itContourPt).x, (*itContourPt).y)) = 255;
            }
            contoursImg.at<uchar>(p) = 255;
        }

        imshow("contours", contoursImg);


        if (imgPts.size() == 4) {
            Point2f centerP;
            imgPts2.clear();
            imgPts2.assign(4, myImgPoint(0, 0));
            centerP = Point2f(((imgPts[0].x + imgPts[1].x + imgPts[2].x + imgPts[3].x) / 4),
                              ((imgPts[0].y + imgPts[1].y + imgPts[2].y + imgPts[3].y) / 4));
            //与中心点形成的角度
            for (int i=0;i<4;i++)
            {
                imgPts[i].angle_with_center=atan2(imgPts[i].y-centerP.y,imgPts[i].x-centerP.x);
            }
            //sort according to angle with center Point
            sort(imgPts.begin(),imgPts.end());
            imgPts[0].length_to_next_point=dist_btn_pts(imgPts[0],imgPts[1]);
            imgPts[1].length_to_next_point=dist_btn_pts(imgPts[1],imgPts[2]);
            imgPts[2].length_to_next_point=dist_btn_pts(imgPts[2],imgPts[3]);
            imgPts[3].length_to_next_point=dist_btn_pts(imgPts[3],imgPts[0]);
            int Index_maxL=0;
            for(int i=1;i<4;i++)
            {
                if(imgPts[i].length_to_next_point>imgPts[Index_maxL].length_to_next_point)
                    Index_maxL=i;
            }
            imgPts2[1]=imgPts[Index_maxL];
            imgPts2[0]=imgPts[(Index_maxL+1)%4];
            imgPts2[3]=imgPts[(Index_maxL+2)%4];
            imgPts2[2]=imgPts[(Index_maxL+3)%4];



//            for (auto itPt = imgPts.begin(); itPt != imgPts.end(); itPt++) {
//                if ((*itPt).x <= centerP.x && (*itPt).y <= centerP.y)
//                    imgPts2[0] = Point2f((*itPt));
//                else if ((*itPt).x > centerP.x && (*itPt).y <= centerP.y)
//                    imgPts2[1] = Point2f((*itPt));
//                else if ((*itPt).x > centerP.x && (*itPt).y > centerP.y)
//                    imgPts2[2] = Point2f((*itPt));
//                else if ((*itPt).x <= centerP.x && (*itPt).y > centerP.y)
//                    imgPts2[3] = Point2f((*itPt));
//            }


            vector<Point3f> objectPts;
            objectPts.emplace_back(-0.0663, -0.03966, 0);
            objectPts.emplace_back(0.06526, -0.03966, 0);
            objectPts.emplace_back(0.03135, 0.06132, 0);
            objectPts.emplace_back(-0.03135, 0.06132, 0);

//            objectPts.emplace_back(-0.0663,0.03966,0);
//            objectPts.emplace_back(0.06526,0.03966,0);
//            objectPts.emplace_back(0.03135,-0.06132,0);
//            objectPts.emplace_back(-0.03135,-0.06132,0);

            Mat rvec, tvec;

            solvePnP(objectPts, imgPts2, cameraMatrix, distCoeffs, rvec, tvec);

//            cout <<  "=========================" << endl;
//            cout <<  "trans" << tvec << endl;
////            cout <<  "rotate" << rvec << endl;
//            cout <<  "=========================" << endl;

            Point ppp;
            ppp = Point(int(tvec.at<double>(0, 0) * 50 + 300), int(tvec.at<double>(0, 1) * 50 + 300));
            if (ppp.x >= 0 && ppp.x <= result.size().width && ppp.y >= 0 && ppp.y <= result.size().height)
                result.at<uchar>(ppp) = 255;

//            imshow("results", result);

//            Mat rot_mat;
//            Rodrigues(Vec3f(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0)), rot_mat);
//            cout << rvec << "   ||||||||" << endl;
//            cout << Vec3f(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0)) << endl;
            Vec3d rvec1 = Vec3d(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
            Vec3d tvec1 = Vec3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
            Affine3f pose(rvec1, tvec1);
//            cout << pose.matrix << endl;
//            myWindow.setWidgetPose("Cube Widget1", pose);
//            myWindow.setWidgetPose("Cube Widget2", pose);

            //相机坐标系修正
//            1.00412352941177	0.614449098039216	0.299829411764706
//            -0.523598775598299	-0.0349065850398866	1.53588974175501
            Matx33f R_x(1, 0, 0,
                        0, cos(THETA_X), -sin(THETA_X),
                        0, sin(THETA_X), cos(THETA_X));
            Matx33f R_y(cos(THETA_Y), 0, sin(THETA_Y),
                        0, 1, 0,
                        -sin(THETA_Y), 0, cos(THETA_Y));
            Matx33f R_z(cos(THETA_Z), -sin(THETA_Z), 0,
                        sin(THETA_Z), cos(THETA_Z), 0,
                        0, 0, 1);
            Matx33f R = R_z * R_x;
            Vec3f tvec_cam = {0, -1, 0.3};
//            Vec3f tvec_cam = {0,0,0};
            Affine3f pose_cam(R, tvec_cam);
//            cout << pose.matrix << endl;
            Affine3f ppposee = pose.concatenate(pose_cam.inv());

            myWindow.setWidgetPose("Cube Widget1", ppposee);
            myWindow.setWidgetPose("Cube Widget2", ppposee);

            myWindow.spinOnce(1, true);
            myfile << tvec1[0] << "," << tvec1[1] << "," << tvec1[2] << "," << tvec1[0] << "," << tvec1[1] << ","
                   << tvec1[2] << "," << endl;
            cout << n << endl;
//            if (n==150)
//                std::getchar();
        }
//        if (n==frames-1)
//        {
//            capture.set(CV_CAP_PROP_POS_FRAMES,1);
//        }
        n++;

        //get the input from the keyboard
        int keyboard = waitKey(50);
        if (keyboard == 'q' || keyboard == 27)
            break;
    }
    myfile.close();
    return 0;
}
