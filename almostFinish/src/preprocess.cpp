#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <iostream>

class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor() : Node("image_processor")
  {
    this->declare_parameter<double>("sigma", 0.5);  // Change to double for fractional values
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&ImageProcessor::topic_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("pre_processed_image", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    double sigma; // Corrected type to double
    this->get_parameter("sigma", sigma);

    cv_bridge::CvImagePtr cv_raw_image;
    try {
      cv_raw_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img_blur;
    cv::GaussianBlur(cv_raw_image->image, img_blur, cv::Size(7,7), sigma, sigma);   
    
auto pre_processed_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_blur).toImageMsg(); 
publisher_->publish(*pre_processed_image);

  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}

