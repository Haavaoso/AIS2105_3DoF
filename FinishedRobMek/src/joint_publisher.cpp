#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Joint_publisher : public rclcpp::Node
{
  public:
    Joint_publisher()
    : Node("joint_publisher")
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("servo_angle_array",10, std::bind(&Joint_publisher::subscriber_callback, this, _1));
    }

  private:
    void subscriber_callback(const std_msgs::msg::Float32MultiArray msg)
    {
      float offset = 90;

      std_msgs::msg::Header header_;
      std::vector<std::string> joint_names_;
      joint_names_.clear();
      joint_names_.push_back("ard_motorA");
      joint_names_.push_back("ard_motorB");
      joint_names_.push_back("ard_motorC");

      trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_;
      trajectory_point_.positions.clear();
      trajectory_point_.positions.push_back(msg.data[0]+offset);
      trajectory_point_.positions.push_back(msg.data[1]+offset);
      trajectory_point_.positions.push_back(msg.data[2]+offset);

      std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points;
      trajectory_points.clear();
      trajectory_points.push_back(trajectory_point_);

      trajectory_msgs::msg::JointTrajectory message;
      message.set__header(header_);
      message.set__joint_names(joint_names_);
      message.set__points(trajectory_points);

      RCLCPP_INFO(this->get_logger(), "Publishing: positions ['%f', '%f', '%f']",trajectory_point_.positions[0], trajectory_point_.positions[1], trajectory_point_.positions[2]);

      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joint_publisher>());
    rclcpp::shutdown();
    return 0;
}
