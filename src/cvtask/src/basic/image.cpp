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
    imshow("Image",img);
    waitKey(0);
    return 0;
}