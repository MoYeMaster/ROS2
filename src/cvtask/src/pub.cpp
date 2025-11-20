#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher(const std::string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点%s启动", name.c_str());
        this->declare_parameter<std::string>("file", "resources/pnp.bmp");//获取文件的路径
        this->get_parameter("file", file);
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ImagePublisher::callback, this));
    }

private:
    std::string file;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void callback()
    {
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv::imread(file, cv::IMREAD_COLOR)).toImageMsg(); // 通过cv_bridge实现opencv和ros图像消息的转换
        RCLCPP_INFO(this->get_logger(), "发送一张图片");
        publisher_->publish(*img_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>("Image_Publish");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}