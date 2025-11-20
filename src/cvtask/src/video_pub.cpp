#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher(const std::string &name) : Node(name)
    {
        this->declare_parameter<std::string>("file", "resources/video.mp4"); // 获取文件的路径
        this->declare_parameter<int>("fps_", 30);//设置视频帧率
        this->declare_parameter<bool>("loop_video", true);
        this->declare_parameter<bool>("publish_as_ros_time", true);
        std::string file = this->get_parameter("file").as_string();
        fps_ = this->get_parameter("fps_").as_int();
        publish_as_ros_time_ = this->get_parameter("publish_as_ros_time").as_bool();
        publish_interval_ms_ = 1000 / fps_;        // 计算发布间隔（毫秒）
        cap.open(path);//打开视频文件
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_interval_ms_), std::bind(&VideoPublisher::callback, this));
        RCLCPP_INFO(this->get_logger(), "节点%s启动", name.c_str());
        RCLCPP_INFO(this->get_logger(), "视频文件: %s", file.c_str());
        RCLCPP_INFO(this->get_logger(), "发布帧率: %d FPS", fps_);
    }

private:
    //声明
    std::string path = "resources/video.mp4";
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;
    int fps_;
    int publish_interval_ms_;
    bool publish_as_ros_time_;

    void callback()
    {
        cv::Mat frame;
        cap.read(frame);
        if (frame.empty())
        {
            RCLCPP_INFO(this->get_logger(),"视频发送完毕，重新发送");
            cap.set(cv::CAP_PROP_POS_FRAMES, 0); // 回到第一帧
            cap.read(frame); // 重新读取第一帧
        }
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg(); // 通过cv_bridge实现opencv和ros图像消息的转换
        publisher_->publish(*img_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPublisher>("Video_Publish");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}