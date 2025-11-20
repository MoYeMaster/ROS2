#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "iostream"

class VideoSubscriber : public rclcpp::Node
{
public:
    VideoSubscriber(const std::string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点%s启动", name.c_str());
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "video",
            10,
            std::bind(&VideoSubscriber::callback, this, std::placeholders::_1));        
        cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);
    }

private:
    cv::Mat img;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void callback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

        video_process(cv_img->image);
    }

    // 对视频进行处理
    void video_process(cv::Mat frame)
    {
        cv::Mat hsv,binary;
        std::vector<cv::Mat> v;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        //转换颜色空间
        cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        //提取亮度通道
        cv::split(hsv, v);
        cv::Mat v_channels = v[2];
        //对图像进行二值化，筛选出亮度高的部分
        cv::threshold(v_channels,binary,210,255,cv::THRESH_BINARY);
        //寻找高亮度区域的轮廓
        cv::findContours(binary,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); i++)
        {
            //绘制边框
            cv::Rect bounding_rect = boundingRect(contours[i]);
            cv::rectangle(frame, bounding_rect, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::imshow("Video",frame);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') { // ESC 或 'q' 键退出
          RCLCPP_INFO(this->get_logger(), "节点关闭");
          rclcpp::shutdown();
          return;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSubscriber>("Video_Process");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}