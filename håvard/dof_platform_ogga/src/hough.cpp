#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
//https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
class HoughTransformNode : public rclcpp::Node
{
public:
    HoughTransformNode() : Node("hough_transform")
    {
        this->declare_parameter<float>("dp", 1);
        this->declare_parameter<float>("min_dist", 480/16);
        this->declare_parameter<int>("param1", 100);
        this->declare_parameter<int>("param2", 30);
        this->declare_parameter<int>("min_radius", 1);
        this->declare_parameter<int>("max_radius", 30);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "canny_edged_image", 10, std::bind(&HoughTransformNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("hough_processed_image", 30);
        position_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("ball_position", 10);

    }

private:
    float x_pos_;
    float y_pos_;
    float z_pos_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {   
        cv::Mat edged_image = cv_bridge::toCvCopy(msg, "mono8")->image;
        cv::Mat black_image = cv::Mat::zeros(edged_image.size(), CV_8UC1);
        std::vector<cv::Vec3f> circles;

        float dp = this->get_parameter("dp").as_double();
        float min_dist = this->get_parameter("min_dist").as_double();
        int param1 = this->get_parameter("param1").as_int();
        int param2 = this->get_parameter("param2").as_int();
        int min_radius = this->get_parameter("min_radius").as_int();
        int max_radius = this->get_parameter("max_radius").as_int();

        //Detekterer sirkler ved hjelp av hough
        cv::HoughCircles(edged_image, circles, cv::HOUGH_GRADIENT, dp, min_dist, param1, param2, min_radius, max_radius);


        //hvis sirker funnet
        if (!circles.empty())
        {
            //Finn senter
            float x = circles[0][0];
            float y = circles[0][1];
            int radius = cvRound(circles[0][2]);

            //Tegn
            cv::Point center(cvRound(x), cvRound(y));
            cv::circle(black_image, center, 1, cv::Scalar(255), 3, cv::LINE_AA);
            cv::circle(black_image, center, radius, cv::Scalar(255), 3, cv::LINE_AA);

            std::cout << "Ball position - X: " << x << ", Y: " << y << std::endl;
            x_pos_ = x;
            y_pos_ = y;
        }
        else
        {
            std::cout << "No circles found." << std::endl;
        }



        auto position_array = std_msgs::msg::Float32MultiArray();
        position_array.data = {x_pos_, y_pos_};

        auto circle_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", black_image).toImageMsg();
        publisher_->publish(*circle_msg);
        position_publisher_->publish(position_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HoughTransformNode>());
    rclcpp::shutdown();
    return 0;
}

