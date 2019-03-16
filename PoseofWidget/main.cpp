#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
using namespace cv;
using namespace std;
static void help()
{
    cout
    << "--------------------------------------------------------------------------"   << endl
    << "This program shows how to visualize a cube rotated around (1,1,1) and shifted "
    << "using Rodrigues vector."                                                      << endl
    << "Usage:"                                                                       << endl
    << "./widget_pose"                                                                << endl
    << endl;
}
int main()
{
    help();
    viz::Viz3d myWindow("Coordinate Frame");
    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
//    viz::WLine axis(Point3f(-1.0f,-1.0f,-1.0f), Point3f(1.0f,1.0f,1.0f));
//    axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
//    myWindow.showWidget("Line Widget", axis);
    viz::WCube cube_widget1(Point3f(-0.0663,-0.03966,-0.01), Point3f(0.0663,0,0.01), false, viz::Color::blue());
    viz::WCube cube_widget2(Point3f(-0.03135,0,-0.01), Point3f(0.03135,0.06132,0.01), false, viz::Color::blue());
//    cube_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);

//    vector<Point3f> objectPts;
//    objectPts.emplace_back(-0.0663,-0.03966,0);
//    objectPts.emplace_back(0.6526,-0.03966,0);
//    objectPts.emplace_back(0.03135,0.06132,0);
//    objectPts.emplace_back(-0.03135,0.06132,0);
//    viz::WCloud cloud_widget(objectPts);

    myWindow.showWidget("Cube Widget1", cube_widget1);
    myWindow.showWidget("Cube Widget2", cube_widget2);
    Mat rot_vec = Mat::zeros(1,3,CV_32F);
    float translation_phase = 0.0, translation = 0.0;
    while(!myWindow.wasStopped())
    {
        /* Rotation using rodrigues */
        rot_vec.at<float>(0,0) += (float)CV_PI * 0.01f;
        rot_vec.at<float>(0,1) += (float)CV_PI * 0.01f;
        rot_vec.at<float>(0,2) += (float)CV_PI * 0.01f;
        translation_phase += (float)CV_PI * 0.01f;
        translation = sin(translation_phase);
        Mat rot_mat;
        Rodrigues(rot_vec, rot_mat);
        Affine3f pose(rot_mat, Vec3f(translation, translation, translation));
        myWindow.setWidgetPose("Cube Widget1", pose);
        myWindow.setWidgetPose("Cube Widget2", pose);
        myWindow.spinOnce(1, true);
    }
    return 0;
}