#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main()
{
    string path = "resources/video.mp4";
    VideoCapture cap(path);
    Mat frame,hsv,binary;
    vector<Mat> v;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    while(true)
    {
        cap.read(frame);    
        //转换颜色空间
        cvtColor(frame,hsv,COLOR_BGR2HSV);
        //提取亮度通道
        split(hsv, v);
        Mat v_channels = v[2];
        //对图像进行二值化，筛选出亮度高的部分
        threshold(v_channels,binary,210,255,THRESH_BINARY);
        //寻找高亮度区域的轮廓
        findContours(binary,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); i++)
        {
            //绘制边框
            Rect bounding_rect = boundingRect(contours[i]);
            rectangle(frame, bounding_rect, Scalar(0, 255, 0), 2);
        }
        
        imshow("Video",frame);
        // imshow("Video",hsv);
        waitKey(30);
    }        
    return 0;
}