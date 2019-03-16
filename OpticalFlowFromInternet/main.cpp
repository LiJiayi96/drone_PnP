

#include <iostream>
#include <fstream>

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

inline bool isFlowCorrect(Point2f u) {
    return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.y) < 1e9;
}

static Vec3b computeColor(float fx, float fy) {
    static bool first = true;

    // relative lengths of color transitions:
    // these are chosen based on perceptual similarity
    // (e.g. one can distinguish more shades between red and yellow
    //  than between yellow and green)
    const int RY = 15;
    const int YG = 6;
    const int GC = 4;
    const int CB = 11;
    const int BM = 13;
    const int MR = 6;
    const int NCOLS = RY + YG + GC + CB + BM + MR;
    static Vec3i colorWheel[NCOLS];

    if (first) {
        int k = 0;

        for (int i = 0; i < RY; ++i, ++k)
            colorWheel[k] = Vec3i(255, 255 * i / RY, 0);

        for (int i = 0; i < YG; ++i, ++k)
            colorWheel[k] = Vec3i(255 - 255 * i / YG, 255, 0);

        for (int i = 0; i < GC; ++i, ++k)
            colorWheel[k] = Vec3i(0, 255, 255 * i / GC);

        for (int i = 0; i < CB; ++i, ++k)
            colorWheel[k] = Vec3i(0, 255 - 255 * i / CB, 255);

        for (int i = 0; i < BM; ++i, ++k)
            colorWheel[k] = Vec3i(255 * i / BM, 0, 255);

        for (int i = 0; i < MR; ++i, ++k)
            colorWheel[k] = Vec3i(255, 0, 255 - 255 * i / MR);

        first = false;
    }

    const float rad = sqrt(fx * fx + fy * fy);
    const float a = atan2(-fy, -fx) / (float) CV_PI;

    const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
    const int k0 = static_cast<int>(fk);
    const int k1 = (k0 + 1) % NCOLS;
    const float f = fk - k0;

    Vec3b pix;

    for (int b = 0; b < 3; b++) {
        const float col0 = colorWheel[k0][b] / 255.f;
        const float col1 = colorWheel[k1][b] / 255.f;

        float col = (1 - f) * col0 + f * col1;

        if (rad <= 1)
            col = 1 - rad * (1 - col); // increase saturation with radius
        else
            col *= .75; // out of range

        pix[2 - b] = static_cast<uchar>(255.f * col);
    }

    return pix;
}

static void drawOpticalFlow(const Mat_<Point2f> &flow, Mat &dst, float maxmotion = -1) {
    dst.create(flow.size(), CV_8UC3);
    dst.setTo(Scalar::all(0));

    // determine motion range:
    float maxrad = maxmotion;

    if (maxmotion <= 0) {
        maxrad = 1;
        for (int y = 0; y < flow.rows; ++y) {
            for (int x = 0; x < flow.cols; ++x) {
                Point2f u = flow(y, x);

                if (!isFlowCorrect(u))
                    continue;

                maxrad = max(maxrad, sqrt(u.x * u.x + u.y * u.y));
            }
        }
    }

    for (int y = 0; y < flow.rows; ++y) {
        for (int x = 0; x < flow.cols; ++x) {
            Point2f u = flow(y, x);

            if (isFlowCorrect(u))
                dst.at<Vec3b>(y, x) = computeColor(u.x / maxrad, u.y / maxrad);
        }
    }
}

int main(int argc, const char *argv[]) {
    Mat frame0;
    Mat frame1;
    Mat_<Point2f> flow;
    Ptr<DenseOpticalFlow> tvl1 = createOptFlow_DualTVL1();
    Mat out;

    if (argc < 2) {
        cerr << "Usage : " << argv[0] << "<video>" << endl;
        return -1;
    }
    VideoCapture cap;
    cap.open(argv[1]);

    while (1) {
        cap >> frame0;
        if (frame0.empty()) {
            cerr << "video is over!!" << endl;
            break;
        }
        cvtColor(frame0, frame0, CV_BGR2GRAY);
        if (!frame1.empty()) {
            const double start = (double) getTickCount();
            tvl1->calc(frame0, frame1, flow);
            const double timeSec = (getTickCount() - start) / getTickFrequency();
            cout << "calcOpticalFlowDual_TVL1 : " << timeSec << " sec" << endl;
            drawOpticalFlow(flow, out);
            imshow("out", out);
            imshow("src", frame0);
            waitKey(10);
        }
        frame0.copyTo(frame1);
    }
    waitKey();

    return 0;
}

