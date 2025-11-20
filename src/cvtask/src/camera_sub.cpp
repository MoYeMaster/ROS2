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

class Camera : public rclcpp::Node
{
public:
    Camera(const std::string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "节点%s启动", name.c_str());
        subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw",
            10,
            std::bind(&Camera::callback, this, std::placeholders::_1));
    }

private:
    cv::Mat img;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void callback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        // cv::imshow("Camera",cv_img->image);
        // cv::waitKey(1);
        pnp_process(cv_img->image);
    }

    void video_process(cv::Mat frame)
    {
        cv::Mat hsv, binary;
        std::vector<cv::Mat> v;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        // 转换颜色空间
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        // 提取亮度通道
        cv::split(hsv, v);
        cv::Mat v_channels = v[2];
        // 对图像进行二值化，筛选出亮度高的部分
        cv::threshold(v_channels, binary, 210, 255, cv::THRESH_BINARY);
        // 寻找高亮度区域的轮廓
        cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); i++)
        {
            // 绘制边框
            cv::Rect bounding_rect = boundingRect(contours[i]);
            cv::rectangle(frame, bounding_rect, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("Video", frame);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q')
        { // ESC 或 'q' 键退出
            RCLCPP_INFO(this->get_logger(), "节点关闭");
            rclcpp::shutdown();
            return;
        }
    }

    void pnp_process(cv::Mat img)
    {
        cv::Mat hsv, binary;
        std::vector<cv::Mat> v;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        // 得到物体的轮廓
        cv::cvtColor(img, hsv, 40);
        cv::split(hsv, v);
        cv::Mat v_channel = v[2];
        cv::threshold(v_channel, binary, 200, 255, 0);
        cv::findContours(binary, contours, hierarchy, 0, 2);

        std::vector<cv::Point2f> point;   // 图片中目标位置的坐标
        std::vector<cv::Point3f> point3d; // 装甲板的数据
        point3d.push_back(cv::Point3f(0, 0, 0));
        point3d.push_back(cv::Point3f(-55, 0, 0));
        point3d.push_back(cv::Point3f(0, 135, 0));
        point3d.push_back(cv::Point3f(-55, 135, 0));

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 2422.61547, 0, 706.68406, 0, 2420.80771, 564.29241, 0, 0, 1); // 相机的内参数矩阵
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.018647, 0.084359, -0.000925, 0.000312, 0.000000);             // 相机的畸变系数

        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Rect bounding_rect = cv::boundingRect(contours[i]);
            rectangle(img, bounding_rect, cv::Scalar(0, 255, 0), 2);
            if (contours.size() >= 2)
            {
                // 获取第一个灯条的坐标(左上和左下)
                if (i == 0)
                {
                    point.push_back(cv::Point2f(bounding_rect.x, bounding_rect.y));
                    point.push_back(cv::Point2f(bounding_rect.br().x - bounding_rect.width, bounding_rect.br().y));
                }
                // 第二个灯条的坐标(右上和右下)
                if (i == 1)
                {
                    point.push_back(cv::Point2f(bounding_rect.x + bounding_rect.width, bounding_rect.y));
                    point.push_back(cv::Point2f(bounding_rect.br().x, bounding_rect.br().y));
                }
            }
            else
                continue;

            // RCLCPP_INFO(this->get_logger(),point[2 * i]);
            // RCLCPP_INFO(this->get_logger(),point[2 * i +1]);
            //  标出四个角的点
            //   circle(img, point[2 * i], 5, cv::Scalar(0, 0, 255), -1);
            //   circle(img, point[2 * i + 1], 5, cv::Scalar(0, 0, 255), -1);
        }
        // 绘制出灯条的外接矩形
        cv::rectangle(img, point[0], point[3], cv::Scalar(0, 69, 200), 2);

        cv::Mat rvec; // 输出的旋转向量
        cv::Mat tvec; // 输出的平移向量

        bool success = cv::solvePnP(point3d, point, camera_matrix, distCoeffs, rvec, tvec);

        if (success)
        {
            // 获取旋转向量和平移向量的结果
            // cv::Mat rotationMatrix;
            // cv::Rodrigues(rvec, rotationMatrix);

            // std::cout << "Rotation Vector: " << std::endl
            //           << rvec << std::endl;
            // std::cout << "Translation Vector: " << std::endl
            //           << tvec << std::endl;
            // std::cout << "Rotation Matrix: " << std::endl
            //           << rotationMatrix << std::endl;
            float distance = sqrt(tvec.at<double>(0, 0) * tvec.at<double>(0, 0) + tvec.at<double>(1, 0) * tvec.at<double>(1, 0) + tvec.at<double>(2, 0) * tvec.at<double>(2, 0)) / 10;
            // std::cout<<"距离: "<<distance<<std::endl;
            // RCLCPP_INFO(this->get_logger(),std::to_string(distance).c_str());
            cv::putText(img, std::to_string(distance), cv::Point(0, 50), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 70, 255), 1); // 将数据打印到图片上
        }
        cv::imshow("pnp", img);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q')
        { // ESC 或 'q' 键退出
            RCLCPP_INFO(this->get_logger(), "节点关闭");
            rclcpp::shutdown();
            return;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera>("camera");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}