#include <chrono>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

// Publisher that sends out the three servo angles.


class Publish_servo_angles : public rclcpp::Node
{
public:
  Publish_servo_angles()
  : Node("servo_publisher"),
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "ball_position", 10, std::bind(&CannyEdgeDetectionNode::timer_callback, this, std::placeholders::_1));

    servo1_angle = this->create_publisher<std_msgs::msg::Float32>("Servo1", 10);
    servo2_angle = this->create_publisher<std_msgs::msg::Float32>("Servo2", 10);
    servo3_angle = this->create_publisher<std_msgs::msg::Float32>("Servo3", 10);
  }

private:
  void timer_callback()
  {
    //Message vector
    auto servo1_ = std_msgs::msg::Float32();
    auto servo2_ = std_msgs::msg::Float32();
    auto servo3_ = std_msgs::msg::Float32();
    
    //RCLCPP_INFO(this->get_logger(), "Publish something: [%f, %f, %f]", message_array.data[0], message_array.data[1], message_array.data[2]);
    servo1_angle->publish(servo1_);
    servo2_angle->publish(servo2_);
    servo3_angle->publish(servo3_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo1_angle;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo2_angle;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo3_angle;

  float x;
  float y;
  float z;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publish_servo_angles>());
  rclcpp::shutdown();
  return 0;
}
