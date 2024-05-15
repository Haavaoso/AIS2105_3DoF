#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <geometry_msgs/msg/point.hpp>

class TransformNode : public rclcpp::Node
{
public:
    TransformNode() : Node("contour_transform")
    {
        // Declare HSV range parameters
        this->declare_parameter<int>("h_min", 20);
        this->declare_parameter<int>("s_min", 180);
        this->declare_parameter<int>("v_min", 120);
        this->declare_parameter<int>("h_max", 25);
        this->declare_parameter<int>("s_max", 255);
        this->declare_parameter<int>("v_max", 255);  
        this->declare_parameter<double>("area", 500); 
        // Subscription to pre-processed images
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "pre_processed_image", 10, std::bind(&TransformNode::image_callback, this, std::placeholders::_1));
        // Publisher for processed images
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("ball_processed_image", 10);
        // Publisher for ball coordinates
        point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ball_coordinates", 10);
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
    	//Retrieve area parameter for Contour
    	double area = this->get_parameter("area").as_double();

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

	// Drawing contours on the original image	
        cv::Mat drawing = color_image.clone();  
	for (const auto& contour : contours) {
	        std::vector<cv::Vec3f> circles;
    		float contour_area = cv::contourArea(contour);
    		if (contour_area > area) { // Increase this threshold to exclude small areas
       			 cv::Point2f center;
        		float radius;
        		cv::minEnclosingCircle(contour, center, radius);
        		if (radius > 10) { // Make sure the radius is sensible
            cv::circle(drawing, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
            std::string coord_text = "(" + std::to_string(static_cast<int>(center.x)) + ", " + std::to_string(static_cast<int>(center.y)) + ")";
            cv::putText(drawing, coord_text, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y) - 10), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
            
            // Publish the ball coordinates
                geometry_msgs::msg::Point point_msg;
                point_msg.x = center.x;
                point_msg.y = center.y;
                point_msg.z = 45.7-((1.9525*540.3052497)/radius); // evt offset = 47.8-1.9525
                point_publisher_->publish(point_msg);
                RCLCPP_INFO(this->get_logger(), "Detected ball at X: %f, Y: %f, Z: %f", center.x, center.y, point_msg.z);
        }
        if (circles.empty()) {
            RCLCPP_WARN(this->get_logger(), "No balls found.");
        }
    }
}
        auto drawing_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", drawing).toImageMsg();
        publisher_->publish(*drawing_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
