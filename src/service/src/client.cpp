#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include "interfaces/srv/process.hpp"

class ServiceClient : public rclcpp::Node
{
public:
    ServiceClient(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点: %s 已启动", name.c_str());
        // 创建客户端
        client_ = this->create_client<interfaces::srv::Process>("image_process_srv");

        // 等待服务
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "等待服务时被中断");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务...");
        }
        RCLCPP_INFO(this->get_logger(), "服务已连接");
        RCLCPP_INFO(this->get_logger(), "发送处理请求...");
        load_image(input_image_);

        RCLCPP_INFO(this->get_logger(), "发送处理请求...");

        auto request = std::make_shared<interfaces::srv::Process::Request>();

        // 将OpenCV图像转换为ROS消息
        auto msg = cv_bridge::CvImage(
                       std_msgs::msg::Header(),
                       "bgr8",
                       input_image_)
                       .toImageMsg();

        request->input_image = *msg;

        // 发送异步请求
        auto future_result = client_->async_send_request(
            request,
            std::bind(&ServiceClient::response_callback, this, std::placeholders::_1));
    }

private:
    // 声明客户端
    rclcpp::Client<interfaces::srv::Process>::SharedPtr client_;
    cv::Mat input_image_;
    std::string path = "resources/pnp.bmp";
    void load_image(cv::Mat &img)
    {
        img = cv::imread(path);
    }

    void response_callback(rclcpp::Client<interfaces::srv::Process>::SharedFuture future)
    {
        auto response = future.get();
    
        if (response->success) {
        RCLCPP_INFO(this->get_logger(), "请求成功: %s", response->message.c_str());
        }
        //将处理完的图片转换回cv格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
          response->processed_image, 
          sensor_msgs::image_encodings::BGR8
        );

        cv::Mat processed_image = cv_ptr->image;
        cv::imshow("Result",processed_image);
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
    auto node = std::make_shared<ServiceClient>("service_client");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
