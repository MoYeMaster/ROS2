#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main()
{
    string path = "resources/video1.mp4";
    VideoCapture cap(path);
    Mat img;

    while(true)
    {
        cap.read(img);
        if (! imshow.empty())
        {
            imshow("Image",img);
            waitKey(20);
        }

    }        
    return 0;
}