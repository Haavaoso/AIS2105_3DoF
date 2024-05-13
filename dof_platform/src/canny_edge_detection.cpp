#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

class CannyEdgeDetectionNode : public rclcpp::Node
{
public:
    CannyEdgeDetectionNode() : Node("canny_edge_detection")
    {
    	// Declare parameters for the Canny edge detection thresholds

        this->declare_parameter<int>("lower_threshold", 20); 
        this->declare_parameter<int>("upper_threshold", 100); 

        // Subscribe to an image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "pre_processed_image", 10, std::bind(&CannyEdgeDetectionNode::image_callback, this, std::placeholders::_1));

            

        // Publish the Canny edge detection image
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("canny_edged_image", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
    	// Get the current threshold values from parameters
        int lower_threshold = this->get_parameter("lower_threshold").as_int();
        int upper_threshold = this->get_parameter("upper_threshold").as_int();

    
    
            cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Apply Canny Edge Detection
            cv::Mat edges;
            cv::Canny(cv_image, edges, lower_threshold, upper_threshold); 

            // Convert the edge-detected image back to a ROS message and publish it
            auto edge_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", edges).toImageMsg();
            publisher_->publish(*edge_msg);
        

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{ 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CannyEdgeDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
