#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main()
{
    string path = "resources/pic1.jpg";
    Mat img = imread(path);
    Mat grayImage,blurImage,cannyImage,dilateImage,erodeImage;
    //生成灰度图
    cvtColor(img,grayImage,COLOR_BGR2GRAY);
    //生成高斯模糊
    GaussianBlur(grayImage,blurImage,Size(7,7),5,0);
    //边缘检测
    Canny(blurImage,cannyImage,50,150);
    //图片扩张和腐蚀
    Mat kernel = getStructuringElement(MORPH_RECT,Size(5,5));//需要一个内核
    dilate(cannyImage,dilateImage,kernel);
    erode(dilateImage,erodeImage,kernel);

    //imshow("Image",img);
    //imshow("GrayImage",grayImage);
    //imshow("BlurImage",blurImage);
    imshow("CannyImage",cannyImage);
    imshow("Dilation Image",dilateImage);
    imshow("Erode Image",erodeImage);

    waitKey(0);

    return 0;
}