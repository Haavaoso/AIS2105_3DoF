#include <vector>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"


class Kinematics_position : public rclcpp::Node
{
public:
  Kinematics_position()
      : Node("camera_kin")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "ball_coordinates", 10, std::bind(&Kinematics_position::position_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("Ball_position_XYZ", 10);
  }

private:
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        //Fra mariannes node
        x = msg->x;
        y = msg->y;
        r = msg->z; //RADIUSSSS


    /*
        Insert code
        morten    
    */

    



    auto array_ = std_msgs::msg::Float32MultiArray();
    array_.data = {x, y, z};
    publisher_->publish(array_);
    }



    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    float x;
    float y;
    float z;
    float r;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Kinematics_position>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}