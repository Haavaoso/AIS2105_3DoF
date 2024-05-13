#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

class HoughTransformNode : public rclcpp::Node
{
public:
    HoughTransformNode() : Node("hough_transform")
    {
        this->declare_parameter<float>("dp", 1);
        this->declare_parameter<float>("min_dist", 100);
        this->declare_parameter<int>("param1", 100);
        this->declare_parameter<int>("param2", 30);
        this->declare_parameter<int>("min_radius", 1);
        this->declare_parameter<int>("max_radius", 30);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "output_edges", 10, std::bind(&HoughTransformNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_circles", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat src = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat gray;
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        cv::medianBlur(gray, gray, 5);

        std::vector<cv::Vec3f> circles;
        float dp = this->get_parameter("dp").as_double();
        float min_dist = this->get_parameter("min_dist").as_double();
        int param1 = this->get_parameter("param1").as_int();
        int param2 = this->get_parameter("param2").as_int();
        int min_radius = this->get_parameter("min_radius").as_int();
        int max_radius = this->get_parameter("max_radius").as_int();

        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, dp, min_dist, param1, param2, min_radius, max_radius);

        for (auto &circle : circles) {
            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
            int radius = cvRound(circle[2]);
            cv::circle(src, center, radius, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        }

        auto circle_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", src).toImageMsg();
        publisher_->publish(*circle_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoughTransformNode>());
    rclcpp::shutdown();
    return 0;
}

