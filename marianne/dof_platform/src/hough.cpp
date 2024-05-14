#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <vector>

class HoughTransformNode : public rclcpp::Node
{
public:
    HoughTransformNode() : Node("hough_transform")
    {
        this->declare_parameter<float>("dp", 1.0);
        this->declare_parameter<float>("min_dist", 480.0);
        this->declare_parameter<int>("param1", 100);
        this->declare_parameter<int>("param2", 75);
        this->declare_parameter<int>("min_radius", 500);
        this->declare_parameter<int>("max_radius", 700);
	this->declare_parameter<int>("canny_minThres",50);
        this->declare_parameter<int>("canny_maxThres",150);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "ball_processed_image", 10, std::bind(&HoughTransformNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("hough_processed_image", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        
float dp = this->get_parameter("dp").as_double();
        float min_dist = this->get_parameter("min_dist").as_double();
        int param1 = this->get_parameter("param1").as_int();
        int param2 = this->get_parameter("param2").as_int();
        int min_radius = this->get_parameter("min_radius").as_int();
        int max_radius = this->get_parameter("max_radius").as_int();
        int canny_minThres = this->get_parameter("canny_minThres").as_int();
        int canny_maxThres = this->get_parameter("canny_maxThres").as_int();

	cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        cv::Mat edges;
        cv::Canny(gray_image, edges, canny_minThres, canny_maxThres);
	
	std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray_image, circles, cv::HOUGH_GRADIENT, dp, min_dist, param1, param2, min_radius, max_radius);

	cv::Mat circle_image = cv_ptr->image.clone();
        for (const auto& circle : circles) {
            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
            int radius = cvRound(circle[2]);
            cv::circle(circle_image, center, radius, cv::Scalar(0,0,255), 3, cv::LINE_AA);
            cv::circle(circle_image, center, 1, cv::Scalar(0,0,255), 3, cv::LINE_AA);
            RCLCPP_INFO(this->get_logger(), "Detected circle at X: %f, Y: %f, Radius: %d", circle[0], circle[1], radius);
        }

        if (circles.empty()) {
            RCLCPP_WARN(this->get_logger(), "No circles found.");
        }
        auto circle_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", circle_image).toImageMsg();
        publisher_->publish(*circle_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HoughTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
   
        
