#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
class ImagePreprocessingNode : public rclcpp::Node
{
public:
    ImagePreprocessingNode() : Node("image_preprocessing_node")
    {
        // Properly initialize it_ after the node construction
        it_ = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());

        // Now, subscribe and advertise using the dereferenced it_
        image_subscriber_ = it_->subscribe("image_raw", 10, &ImagePreprocessingNode::image_callback, this);
        image_publisher_ = it_->advertise("processed_image", 10);
    }

    ~ImagePreprocessingNode()
    {
        // Destructor to ensure cleanup if necessary, though unique_ptr handles it automatically
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            // Convert the ROS image message to an OpenCV image
            cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;

            // Convert the processed OpenCV image back to a ROS image message
            auto out_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, cv_image).toImageMsg();

            // Publish the processed image
            image_publisher_.publish(*out_msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_subscriber_;
    image_transport::Publisher image_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePreprocessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

