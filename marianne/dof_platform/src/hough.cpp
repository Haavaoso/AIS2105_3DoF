#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

class HoughTransformNode : public rclcpp::Node
{
public:
    HoughTransformNode() : Node("hough_transform")
    {
        // Declare HSV range parameters
        this->declare_parameter<int>("h_min", 10);
        this->declare_parameter<int>("s_min", 150);
        this->declare_parameter<int>("v_min", 50);
        this->declare_parameter<int>("h_max", 25);
        this->declare_parameter<int>("s_max", 255);
        this->declare_parameter<int>("v_max", 255);  
        // Subscription to pre-processed images
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "pre_processed_image", 10, std::bind(&HoughTransformNode::image_callback, this, std::placeholders::_1));
        // Publisher for processed images
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("hough_processed_image", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
    cv_bridge::CvImagePtr cv_ptr; // Declare cv_ptr outside of try block
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return; // Early return if conversion fails
    }

        cv::Mat color_image = cv_ptr->image;
        cv::Mat hsv_image;
        cv::cvtColor(color_image, hsv_image, cv::COLOR_BGR2HSV);

        // Retrieve HSV range parameters
        int h_min = this->get_parameter("h_min").as_int();
        int s_min = this->get_parameter("s_min").as_int();
        int v_min = this->get_parameter("v_min").as_int();
        int h_max = this->get_parameter("h_max").as_int();
        int s_max = this->get_parameter("s_max").as_int();
        int v_max = this->get_parameter("v_max").as_int();
        cv::Scalar lower_orange(h_min, s_min, v_min);
        cv::Scalar upper_orange(h_max, s_max, v_max);
        
        // Mask creation for color filtering
        cv::Mat mask;
        cv::inRange(hsv_image, lower_orange, upper_orange, mask);
        
        // Noise reduction through erosion and dilation
        cv::Mat eroded, dilated;
        cv::erode(mask, eroded, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(eroded, dilated, cv::Mat(), cv::Point(-1, -1), 2);
        
        // Contour detection and simplification
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat drawing = color_image.clone();  // Drawing contours on the original image
        for (const auto& contour : contours) {
            float contour_area = cv::contourArea(contour);
            if (contour_area > 100) {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contour, center, radius);
                cv::circle(drawing, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                std::string coord_text = "(" + std::to_string(static_cast<int>(center.x)) + ", " + std::to_string(static_cast<int>(center.y)) + ")";
                cv::putText(drawing, coord_text, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
            }
        }

        auto drawing_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", drawing).toImageMsg();
        publisher_->publish(*drawing_msg);
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

