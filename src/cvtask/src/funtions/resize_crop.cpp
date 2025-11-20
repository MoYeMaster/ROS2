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
    Mat imgResize,imgCrop;

    cout<<img.size()<<endl;
    //调整图片的大小
    //resize(image,image,Size(num,num))可以直接调整图片大小
    resize(img,imgResize,Size(),0.5,0.5);//按比例缩放
    //裁减图片
    Rect roi(100,100,1000,500);//矩形左上角坐标和矩形的宽度和高度
    imgCrop = img(roi);

    imshow("Image",imgResize);
    imshow("ImageCrop",imgCrop);
    waitKey(0);

    return 0;
}