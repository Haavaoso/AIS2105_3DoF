#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <opencv2/opencv.hpp>

class CoordinateTransformNode : public rclcpp::Node
{
public:
    CoordinateTransformNode() : Node("coordinate_transform_node")
    {
        // Load camera calibration parameters from ROS parameters or set defaults
        this->declare_parameter<double>("fx", 525.0);
        this->declare_parameter<double>("fy", 525.0);
        this->declare_parameter<double>("cx", 319.5);
        this->declare_parameter<double>("cy", 239.5);
        this->get_parameter("fx", fx);
        this->get_parameter("fy", fy);
        this->get_parameter("cx", cx);
        this->get_parameter("cy", cy);

        camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F); // Assuming no lens distortion for simplicity

        // Subscriber to the feature points topic
        feature_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "feature_points", 10, std::bind(&CoordinateTransformNode::feature_callback, this, std::placeholders::_1));

        // Publisher for the transformed coordinates
        transformed_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("transformed_points", 10);
    }

private:
    void feature_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Convert pixel coordinates to physical coordinates using the camera matrix
        cv::Point2f pixel_point(msg->point.x, msg->point.y);
        cv::Point3f world_point;

        // Assuming a simple pinhole camera model for demonstration purposes
        world_point.x = (pixel_point.x - cx) / fx;
        world_point.y = (pixel_point.y - cy) / fy;
        world_point.z = 1; // Z-coordinate can be set based on specific use-case or depth information

        // Retrieve or compute theta here (example uses a placeholder)
        double theta = get_rotation_angle(); // Implement or properly reference this function

        // Apply rotation matrix
        float cos_theta = cos(theta);
        float sin_theta = sin(theta);
        float rotated_x = world_point.x * cos_theta - world_point.y * sin_theta;
        float rotated_y = world_point.x * sin_theta + world_point.y * cos_theta;

        // Publish the transformed and rotated point
        geometry_msgs::msg::PointStamped transformed_msg;
        transformed_msg.header.stamp = this->get_clock()->now();
        transformed_msg.point.x = rotated_x;
        transformed_msg.point.y = rotated_y;
        transformed_msg.point.z = world_point.z;
        transformed_publisher_->publish(transformed_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr feature_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr transformed_publisher_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double fx; // Focal length x
    double fy; // Focal length y
    double cx; // Principal point x
    double cy; // Principal point y

    double get_rotation_angle()
    {
        // Placeholder implementation, adjust as needed based on your application
        return 0.0; // No rotation by default
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinateTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

