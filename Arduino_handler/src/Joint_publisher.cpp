#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

class Joint_publisher : public rclcpp::Node
{
  public:
    Joint_publisher()
    : Node("joint_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);


      timer_ = this->create_wall_timer(
      50ms, std::bind(&Joint_publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      std_msgs::msg::Header header_;
      std::vector<std::string> joint_names_;
      joint_names_.clear();
      joint_names_.push_back("ard_motorA");
      joint_names_.push_back("ard_motorB");
      joint_names_.push_back("ard_motorC");

      trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_;
      trajectory_point_.positions.clear();
      trajectory_point_.positions.push_back(sinf(count_+=0.1)*45+90);
      trajectory_point_.positions.push_back(sinf(count_+=0.1+3.141592*2/3)*45+90);
      trajectory_point_.positions.push_back(sinf(count_+=0.1+3.141592*4/3)*45+90);

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
    float count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Joint_publisher>());
    rclcpp::shutdown();
    return 0;
}