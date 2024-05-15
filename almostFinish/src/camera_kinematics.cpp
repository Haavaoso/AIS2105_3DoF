#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class Kinematics_position : public rclcpp::Node
{
  public:
  Kinematics_position() : Node("camera_kin")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "ball_coordinates", 10, std::bind(&Kinematics_position::pixel_callback, this, std::placeholders::_1));

    subscription_pid = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "pitch_n_roll", 10, std::bind(&Kinematics_position::position_callback2, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("Ball_position_XYZ", 10);
    
  
        camera_matrix_ = cv::Matx33f(635.463155, 0, 346.312310,
                                     0, 636.244545, 226.047659,
                                     0, 0, 1);
    }

private:
    	void position_callback2(const std_msgs::msg::Float32MultiArray::SharedPtr pitch_n_roll){
        float gamma1 = pitch_n_roll->data[0];
  	float theta = pitch_n_roll->data[1];
  	}
    void pixel_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        cv::Matx31f pixel_cord(msg->x, msg->y, 1);
        cv::Matx31f world_cord = camera_matrix_.inv() * pixel_cord;
        double z_world = msg->z; 

        world_cord *= z_world;

        // Create and publish the converted point
        auto point_msg = geometry_msgs::msg::Point();
        point_msg.x = world_cord(0, 0);
        point_msg.y = world_cord(1, 0);
        point_msg.z = world_cord(2, 0);

        RCLCPP_INFO(this->get_logger(), "Converted to world coordinates: (%f, %f, %f)", point_msg.x, point_msg.y, point_msg.z);
        publisher_->publish(point_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_pid;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    cv::Matx33f camera_matrix_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Kinematics_position>());
    rclcpp::shutdown();
    return 0;
}
