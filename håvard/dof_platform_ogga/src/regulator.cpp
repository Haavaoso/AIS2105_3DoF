
#define _USE_MATH_DEFINES

#include <memory>

#include <ostream>
#include <chrono>     
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using std::placeholders::_1;

#include "regulator_functions.cpp"

// hyp = 22.5
// normal = 19.5
int setpoint[3] = {313, 278,
 0};
float ball_pos[3] = {1, 0, 0};


double z_tuning = + 0;
double z_ = 10+z_tuning;

//Maxmin output value of pid for degree conversion


float pid_p = 0.1;
float pid_i = 0.02;
float pid_d = 0.01;

// Maxminpitch possible by 3dof platform
int max_pitch = 5;
int min_pitch = -5;
int max_roll = 5;
int min_roll = -5;// Driver code

float servo1_ang = 90;
float servo2_ang = 90;
float servo3_ang = 90;


PID PID_X(pid_p, pid_i, pid_d);
PID PID_Y(pid_p, pid_i, pid_d);


class Platform_Regulator : public rclcpp::Node
{
public:
  Platform_Regulator()
      : Node("regulator")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "ball_position", 10, std::bind(&Platform_Regulator::topic_callback, this, _1));
  
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("servo_angle_array", 100);
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Coordinates recivced: [%f, %f, %f]", msg->data[0], msg->data[1], msg->data[2]);
    float pidx_ = PID_X.compute(setpoint[0], msg->data[0]);
    float pidy_ = PID_Y.compute(setpoint[1], msg->data[1]);

    std::cout << "PIDX: " << pidx_ << " PIDY: " << pidy_ << std::endl;

    // Map output of PID to table tilt
    float pitch_deg = min_pitch + ((max_pitch - min_pitch) / (max_pid - min_pid)) * (pidx_ - min_pid);
    float roll_deg = min_roll + ((max_roll - min_roll) / (max_pid - min_pid)) * (pidy_ - min_pid);


    std::cout << "pitch: " << pitch_deg << " roll_deg: " << roll_deg << std::endl;
    std::vector<double> calculations_deg = next_servo_pos(pitch_deg, roll_deg, z_);
    std::vector<double> anglee = servo_angle_outputs(calculations_deg);
    
    RCLCPP_INFO(this->get_logger(), "PID IS CRAZY: [%f, %f]", pidy_, pidx_);
    RCLCPP_INFO(this->get_logger(), "ROLL IS CRAZY: [%f, %f, %f]", anglee[0], anglee[1], anglee[2]);


  
    auto array_ = std_msgs::msg::Float32MultiArray();
    array_.data = {anglee[0], anglee[1], anglee[2]};
    publisher_->publish(array_);
  }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  float x;
  float y;
  float z;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Platform_Regulator>());
  rclcpp::shutdown();
  return 0;
}


