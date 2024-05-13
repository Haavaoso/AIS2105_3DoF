#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor() : Node("image_processor")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&ImageProcessor::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_clone", 10);
    cv::namedWindow("Raw image for cam");
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_raw_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    //klon bilde 
    cv::Mat image_clone = cv_raw_image->image.clone();

    //Sender prossesert bilde 
    auto cloned_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_clone).toImageMsg();
    publisher_->publish(*cloned_image_msg);
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
