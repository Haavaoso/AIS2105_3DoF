#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

class HoughTransformNode : public rclcpp::Node
{
public:
    HoughTransformNode() : Node("hough_transform")
    {
        this->declare_parameter<float>("dp", 1.0);
        this->declare_parameter<float>("min_dist", 480.0);
        this->declare_parameter<int>("param1", 100);
        this->declare_parameter<int>("param2", 30);
        this->declare_parameter<int>("min_radius", 1);
        this->declare_parameter<int>("max_radius", 30);
	this->declare_parameter<int>("canny_minThres",50);
        this->declare_parameter<int>("canny_maxThres",150);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "pre_processed_image", 10, std::bind(&HoughTransformNode::image_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("hough_processed_image", 10);
    }
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat edged_image = cv_ptr->image;
        std::vector<cv::Vec3f> circles;

        float dp = this->get_parameter("dp").as_double();
        float min_dist = this->get_parameter("min_dist").as_double();
        int param1 = this->get_parameter("param1").as_int();
        int param2 = this->get_parameter("param2").as_int();
        int min_radius = this->get_parameter("min_radius").as_int();
        int max_radius = this->get_parameter("max_radius").as_int();
        int canny_minThres = this->get_parameter("canny_minThres").as_int();
        int canny_maxThres = this->get_parameter("canny_maxThres").as_int();

        cv::Mat edges;
        cv::Canny(edged_image, edges, canny_minThres, canny_maxThres);
        cv::HoughCircles(edges, circles, cv::HOUGH_GRADIENT, dp, min_dist, param1, param2, min_radius, max_radius);

        cv::Mat circle_image = cv::Mat::zeros(edges.size(), CV_8UC1);

	if (!circles.empty())
        {
            // Extract the first detected circle's parameters
            float x = circles[0][0];  // X coordinate of the center
            float y = circles[0][1];  // Y coordinate of the center
            int radius = cvRound(circles[0][2]);  // Radius of the circle

            // Draw the detected circle and its center
            cv::Point center(cvRound(x), cvRound(y));
            cv::circle(circle_image, center, 1, cv::Scalar(255), 3, cv::LINE_AA);
            cv::circle(circle_image, center, radius, cv::Scalar(255), 3, cv::LINE_AA);

            // Report the position of the ball
            std::cout << "Ball position - X: " << x << ", Y: " << y << std::endl;
        }
        else
        {
            std::cout << "No circles found." << std::endl;
        }


        auto circle_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", circle_image).toImageMsg();
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

