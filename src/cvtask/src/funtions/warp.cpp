#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

float w = 250,h = 350;
Mat matrix,imgWarp;

int main()
{
    string path = "resources/pic2.png";
    Mat img= imread(path);

    Point2f src[4] = {{553,149},{811,198},{711,477},{424,414}};
    Point2f dst[4] = {{0.0f,0.0f},{w,0.0f},{w,h},{0.0f,h}};

    //透视变换
    matrix = getPerspectiveTransform(src,dst);
    warpPerspective(img,imgWarp,matrix,Point(w,h));

    for (int i = 0; i < sizeof(src); i++)
    {
        circle(img,Point(src[i]),10,Scalar(0,0,0),2);
    }
    

    imshow("Image",img);
    imshow("ImageWarp",imgWarp);
    waitKey(0);
    return 0;
}