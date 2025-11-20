#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture cap(1);
    Mat img;

    while(true)
    {
        cap.read(img);
        imshow("Image",img);
        waitKey(1);

    }        
    return 0;
}