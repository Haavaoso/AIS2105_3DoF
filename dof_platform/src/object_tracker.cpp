#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

class ObjectTrackerNode : public rclcpp::Node {
public:
    ObjectTrackerNode() : Node("object_tracker_node") {
        image_transport::ImageTransport it(shared_from_this());
        image_subscriber_ = it.subscribe("output_circles", 10, &ObjectTrackerNode::image_callback, this);
        tracker_ = cv::TrackerKCF::create(); }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Rect2d bbox = cv::Rect2d(250, 250, 0, 1);  // Example starting position and size


            if (tracker_->update(frame, bbox,frame &)) {
                // Process tracking data
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    image_transport::Subscriber image_subscriber_;
    cv::Ptr<cv::Tracker> tracker_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

