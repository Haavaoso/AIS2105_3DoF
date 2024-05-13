#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <iostream>


class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor() : Node("image_processor")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&ImageProcessor::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("pre_processed_image", 10);
    this->declare_parameter<int>("sigma", 3);

  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    int sigma = this->get_parameter("sigma").as_int();



    cv_bridge::CvImagePtr cv_raw_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    //klon bilde 
    cv::Mat working_img = cv_raw_image->image.clone();

    //Koverter til grå
    cv::Mat img_gray;
    cv::cvtColor(working_img, img_gray, cv::COLOR_BGR2GRAY);

    cv::Mat img_blur;
    cv::GaussianBlur(img_gray, img_blur, cv::Size(9,9), sigma, sigma);
    
  
    //Sender prossesert bilde 
    auto pre_processed_image = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img_blur).toImageMsg(); 
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
