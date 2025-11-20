#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int hmin = 0,smin = 110,vmin = 153;
int hmax = 19,smax = 240,vmax = 250;

int main()
{
    string path = "resources/pic3.png";
    Mat img = imread(path),imgHSV,mask;

    cvtColor(img,imgHSV,COLOR_BGR2HSV);//颜色空间的转换

    namedWindow("TrackBars");
    createTrackbar("H Min","TrackBars",&hmin,179);
    createTrackbar("H Max","TrackBars",&hmax,179);
    createTrackbar("S Min","TrackBars",&smin,255);
    createTrackbar("S Max","TrackBars",&smax,255);
    createTrackbar("V Min","TrackBars",&vmin,255);
    createTrackbar("V Max","TrackBars",&vmax,255);


    while(true)
    {
    Scalar lower(hmin,smin,vmin),upper(hmax,smax,vmax);
    inRange(imgHSV,lower,upper,mask);//选取一个颜色范围

    imshow("Image",img);
    //imshow("Image HSV",imgHSV);
    imshow("Image Mask",mask);
    waitKey(1);
    }    
    return 0;

}