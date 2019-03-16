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


using namespace cv;
using namespace std;



int main(int argc, char *argv[]) {



    VideoCapture cap("/home/lijiayi/CLionProjects/VideoFiles/GOPR2211.MP4");

    Mat frame,img,valueImg,hsvImg,binImg,rectImg,result;
    int frame_width=cap.get(CAP_PROP_FRAME_WIDTH);
    int frame_height=cap.get(CAP_PROP_FRAME_HEIGHT);

    VideoWriter contoursVideo("contoursVideo1.avi",CV_FOURCC('M','J','P','G'),30, Size(frame_width,frame_height),false);
    VideoWriter resultVideo("resultVideo1.avi",CV_FOURCC('M','J','P','G'),30, Size(1000,1000),false);
    Mat out;


    //相机参数
    Mat cameraMatrix;
    cameraMatrix=Mat::zeros(3,3,CV_64F);
    const vector<double> distCoeffs{-2.6522551789694315e-01,9.3921727862528878e-02,0,0,-1.7294295037019555e-02};

    cameraMatrix.at<double>(0,0)=9.0192543754638007e+02;
    cameraMatrix.at<double>(0,2)=960;
    cameraMatrix.at<double>(1,1)=9.0192543754638007e+02;
    cameraMatrix.at<double>(1,2)=540;
    cameraMatrix.at<double>(2,2)=1;

    namedWindow("undistorted", WINDOW_NORMAL);
    cap>> frame;
    img=Mat::zeros(frame.size(),frame.type());
    valueImg=Mat::zeros(img.size(),CV_8UC1);

    result=Mat::zeros(1000,1000,CV_8UC1);

    namedWindow("Binary", WINDOW_NORMAL);
    namedWindow("contours", WINDOW_NORMAL);
    namedWindow("Dilated", WINDOW_NORMAL);
    namedWindow("results", WINDOW_NORMAL);

    //region 3D virtualization
    viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
    viz::WCube cube_widget1(Point3f(-0.0663,-0.03966,-0.01), Point3f(0.0663,0,0.01), false, viz::Color::blue());
    viz::WCube cube_widget2(Point3f(-0.03135,0,-0.01), Point3f(0.03135,0.06132,0.01), false, viz::Color::blue());
    myWindow.showWidget("Cube Widget1", cube_widget1);
    myWindow.showWidget("Cube Widget2", cube_widget2);
    myWindow.spinOnce(1, true);

//endregion


    for(;;){
        if(cap.read(frame))
        {
            undistort(frame, img, cameraMatrix, distCoeffs);
            imshow("undistorted",img);

            cvtColor(img,hsvImg,CV_BGR2HSV);

            for (int i = 0; i < img.rows; i++) {
                for (int j = 0; j < img.cols; j++) {
                    const int hi = i * img.cols * 3 + j * 3,
                            gi = i * img.cols + j;
                    valueImg.data[gi] = hsvImg.data[hi + 2];
                }
            }


            threshold(valueImg, binImg,250,255,THRESH_BINARY);

            imshow("Binary", binImg);
            //region 膨胀
            Mat dilatedImg;
            int structElementSize = 1;
            Mat structElement = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1),
                                                      Point(structElementSize, structElementSize));
            dilate(binImg, dilatedImg, structElement);
            imshow("Dilated", dilatedImg);
            //endregion

            //region 寻找轮廓
            vector<vector<Point>> contours;

            vector<Vec4i> hierarchy;
            findContours(dilatedImg, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

            Mat contoursImg = Mat::zeros(img.size(), CV_8UC1);
//          rectImg = img.clone();

//            drawContours(contoursImg, contours, -1, Scalar(255), 3, 8, hierarchy);

//endregion
            vector<Point2f> imgPts,imgPts2;

            for (auto itContour = contours.begin(); itContour != contours.end(); itContour++)
            {
                if(contourArea(*itContour)<40)
                {
                    continue;
                }

                Moments m;
                m=moments(*itContour,true);
                Point2f p=Point2f((m.m10/m.m00),(m.m01/m.m00));
                imgPts.push_back(p);

                for(auto itContourPt=(*itContour).begin();itContourPt!=(*itContour).end();itContourPt++)
                {
                    contoursImg.at<uchar>(Point((*itContourPt).x,(*itContourPt).y))=255;
                }
                contoursImg.at<uchar>(p)=255;
            }

            imshow("contours", contoursImg);
            contoursVideo.write(contoursImg);


            if(imgPts.size()==4)
            {
                Point2f centerP;
                imgPts2.clear();
                imgPts2.assign(4,Point2f(0,0));
                centerP=Point2f(((imgPts[0].x+imgPts[1].x+imgPts[2].x+imgPts[3].x)/4),((imgPts[0].y+imgPts[1].y+imgPts[2].y+imgPts[3].y)/4));
                for(auto itPt=imgPts.begin();itPt!=imgPts.end();itPt++)
                {
                    if((*itPt).x<=centerP.x && (*itPt).y<=centerP.y)
                        imgPts2[0]=Point2f((*itPt));
                    else if((*itPt).x>centerP.x && (*itPt).y<=centerP.y)
                        imgPts2[1]=Point2f((*itPt));
                    else if((*itPt).x>centerP.x && (*itPt).y>centerP.y)
                        imgPts2[2]=Point2f((*itPt));
                    else if((*itPt).x<=centerP.x && (*itPt).y>centerP.y)
                        imgPts2[3]=Point2f((*itPt));
                }


            vector<Point3f> objectPts;
            objectPts.emplace_back(0,0,0);
            objectPts.emplace_back(0,0.065,0);
            objectPts.emplace_back(0.114,0.113,0);
            objectPts.emplace_back(0.114,0,0);

            Mat rvec,tvec;

            solvePnP(objectPts,imgPts2,cameraMatrix,distCoeffs, rvec, tvec);

            cout <<  "=========================" << endl;

            cout <<  "trans" << tvec << endl;
            cout <<  "rotate" << rvec << endl;
            cout <<  "=========================" << endl;

                Point ppp;
            ppp=Point(int(tvec.at<double>(0,0)*250+300),int(tvec.at<double>(0,1)*250+300));
            result.at<uchar>(ppp)=255;

            imshow("results", result);
            resultVideo.write(result);

                Mat rot_mat;
                Rodrigues(Vec3f(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0)), rot_mat);
//            cout << rvec << "   ||||||||" << endl;
//            cout << Vec3f(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0)) << endl;
                Affine3f pose(Vec3f(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0)),
                              Vec3f(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0)));
                cout << pose.matrix << endl;
                myWindow.setWidgetPose("Cube Widget1", pose);
                myWindow.setWidgetPose("Cube Widget2", pose);
                myWindow.spinOnce(1, true);

            }
        }
        if (waitKey(1) >= 0) break;
    }

    return 0;
}