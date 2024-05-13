#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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
        this->declare_parameter<float>("min_dist", 480/1);
        this->declare_parameter<int>("param1", 100);
        this->declare_parameter<int>("param2", 30);
        this->declare_parameter<int>("min_radius", 1);
        this->declare_parameter<int>("max_radius", 30);

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "canny_edged_image", 10, std::bind(&HoughTransformNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("hough_processed_image", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat black_image = cv::Mat::zeros(480, 640, CV_8UC1);
        cv::Mat edged_image = cv_bridge::toCvCopy(msg, "mono8")->image;
        std::vector<cv::Vec3f> circles;

        float dp = this->get_parameter("dp").as_double();
        float min_dist = this->get_parameter("min_dist").as_double();
        int param1 = this->get_parameter("param1").as_int();
        int param2 = this->get_parameter("param2").as_int();
        int min_radius = this->get_parameter("min_radius").as_int();
        int max_radius = this->get_parameter("max_radius").as_int();

        //Detekterer sirkler ved hjelp av hough
        cv::HoughCircles(edged_image, circles, cv::HOUGH_GRADIENT, dp, min_dist, param1, param2, min_radius, max_radius);


            ////tegner sirklene på bilde
            //for (auto &circle : circles) {
            //    cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
            //    int radius = cvRound(circle[2]);
            //    cv::circle(src, center, radius, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
            //}


        
        for( size_t i = 0; i < circles.size(); i++ )
        {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);

            // circle center
            cv::circle(black_image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);

            // circle outline
            int radius = c[2];
            cv::circle(black_image, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_8);
        }





        auto circle_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", black_image).toImageMsg();
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

