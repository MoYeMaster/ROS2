#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

//寻找轮廓
void getContours(Mat imgDil,Mat imgDraw)
{
    vector<vector<Point>> contours;//轮廓
    vector<Vec4i> hierarchy;//层次结构
    findContours(imgDil,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    //遍历每个轮廓检测大小
    for (int i = 0; i < contours.size(); i++)
    {
        int area = contourArea(contours[i]);
        cout<<area<<endl;

        vector<vector<Point>> conPoly(contours.size());
        vector<Rect> boundRect(contours.size());

        string objType;

        if (area >= 1000)
        {
            //进行形状检测
            float peri = arcLength(contours[i],true /*是否封闭*/);
            approxPolyDP(contours[i],conPoly[i],0.02*peri,true);//轮廓分析
            boundingRect(conPoly[i]);

            int objCor = (int)conPoly[i].size();//获取顶点数量

            if (objCor == 3)
            {
                objType = "Tri";
            }else if (objCor == 4)
            {
                float asp = (float)boundRect[i].width / (float)boundRect[i].height;
                if (asp > 0.9 && asp < 1,1)
                {
                    objType = "Square";
                }
                else{objType = "Rect";}
            }else if (objCor > 6)
            {
                objType = "Circle";
            }

            rectangle(imgDraw,boundRect[i].tl(),boundRect[i].br(),Scalar(0,255,255),5);
            putText(imgDraw,objType,{boundRect[i].x,boundRect[i].y - 5},FONT_HERSHEY_COMPLEX,1,Scalar(0,70,255),2);//两个数字分别是字体大小和字体厚度
            drawContours(imgDraw,conPoly,i,Scalar(0,0,255),2);
            
        }
    }
}

int main()
{
    string path = "resources/pic4.png";
    Mat img = imread(path);
    Mat imgGray,imgBlur,imgCanny,imgDil;

    //图片的预处理
    //边缘检测
    cvtColor(img,imgGray,COLOR_BGR2GRAY);
    GaussianBlur(imgGray,imgBlur,Size(7,7),5,0);
    Canny(imgBlur,imgCanny,50,150);
    //扩张
    Mat kernel = getStructuringElement(MORPH_RECT,Size(5,5));
    dilate(imgCanny,imgDil,kernel);

    getContours(imgDil,img);


    imshow("Image",img);
    waitKey(0);
    return 0;
}