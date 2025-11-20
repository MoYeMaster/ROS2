#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main()
{
    //创建一张空白图像
    Mat canva(512,512,CV_8UC3,Scalar(255,255,255));

    //绘制图形
    circle(canva,Point(256,256),150,Scalar(0,0,255),-1);//图片，中心点，大小，颜色，厚度
    rectangle(canva,Point(130,226),Point(382,286),Scalar(255,255,255),FILLED);
    line(canva,Point(130,296),Point(380,296),Scalar(255,255,255),3);

    //输出文字
    putText(canva,"MoYe",Point(137,262),FONT_HERSHEY_COMPLEX,1,Scalar(0,70,255),2);//两个数字分别是字体大小和字体厚度

    imshow("Canva",canva);
    waitKey(0);

    return 0;
}