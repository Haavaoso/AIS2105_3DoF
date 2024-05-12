
#define _USE_MATH_DEFINES

#include <memory>
#include <vector>
#include <cmath>
#include <ostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using std::placeholders::_1;


//int heve = 1; // MÅ SELTTESTo
//double elbow_theta = m1M_PI - std::acos((-x * x - y * y + L_1 * L_1 + L_2 * L_2) / (2 * L_1 * L_2));
//float y = heve; // Hennt fra ny posisjons matrise

// hyp = 22.5
// normal = 19.5
float L_1 = 10.5;
float L_2 = 4.5;
float x = 1;    // Avstand fra tilkoplingspunkt plate til servo aktuator i x-planet


void multiply(int m1, int m2, int mat1[][2], int n1, int n2,
              int mat2[][2])
{
    int x, i, j;
    int res[m1][n2];
    for (i = 0; i < m1; i++) {
        for (j = 0; j < n2; j++) {
            res[i][j] = 0;
            for (x = 0; x < m2; x++) {
                *(*(res + i) + j) += *(*(mat1 + i) + x)
                                     * *(*(mat2 + x) + j);
            }
        }
    }
    for (i = 0; i < m1; i++) {
        for (j = 0; j < n2; j++) {
            std::cout << *(*(res + i) + j) << " ";
        }
        std::cout << "\n";
    }
}

// Driver code
int main()
{
    int mat1[][2] = { { 1, 1 }, { 2, 2 } };
    int mat2[][2] = { { 1, 1 }, { 2, 2 } };
    int m1 = 2, m2 = 2, n1 = 2, n2 = 2;

    // Function call
    multiply(m1, m2, mat1, n1, n2, mat2);
    return 0;
}




class PID
{
private:
  double kp, ki, kd;
  bool integral_limit = false;
  int integral_cap = 20;
  double integral; // Integral accumulator
  double previous_error;
  std::chrono::steady_clock::time_point last_time;

public:
  PID(double p, double i, double d) : kp(p), ki(i), kd(d), integral(0.0), previous_error(0.0)
  {
    last_time = std::chrono::steady_clock::now();
  }

  double compute(double setpoint, double ball_pos)
  {
    // Current time
    auto now = std::chrono::steady_clock::now();

    // Calculate dt
    std::chrono::duration<double> dt = now - last_time;
    double delta_time = dt.count();
    delta_time = 0.5;
    // Calculate the error
    double error = setpoint - ball_pos;

    // Integral term
    integral += error * delta_time;

    // Integral windup protection
    if (integral_limit)
    {
      if (integral > integral_cap)
      {
        integral = integral_cap;
      }
      else if (integral < -integral_cap)
      {
        integral = -integral_cap;
      }
    }
    // Derivative term
    double derivative = (error - previous_error) / delta_time;
    // PID output
    double output = kp * error + ki * integral + kd * derivative;

    // Update previous error and time
    previous_error = error;
    last_time = now;

    return output;
  }
};

// Maxmin servo angles
float servo_angle_limit_max = 85;
float servo_angle_limit_min = -35;

float servo1_ang = 90;
float servo2_ang = 90;
float servo3_ang = 90;

// Maxminpitch possible by 3dof platform
int max_pitch = 5;
int min_pitch = -5;
int max_roll = 5;
int min_roll = -5;

double PID_P = 0.2;
double PID_I = 0.01;
double PID_D = 0.01;
PID PID_X(PID_P, PID_I, PID_D);
PID PID_Y(PID_P, PID_I, PID_D);MultiplyWithTiling

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "topic2", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  float L = 22.5;
  double h = L * std::sqrt(3 / 4);
  int z_ = 2;
  float gamma = 0;
  float theta = 0;

  double servo_ang[3];
  double P_roti[4][4] = 
       {{std::sin(gamma) * std::sin(theta), std::sin(gamma) * std::cos(theta), 0},
       {0, std::cos(theta), -std::sin(theta), 0},
       {-std::sin(gamma), std::cos(gamma) * std::sin(theta), 0, z_},
       {0, 0, 0, 1}};

  double pos1[4][1] =
      {{-L / 2},
       {-h / 3},
       {0},
       {1}};
  double pos2[4][1] =
      {{L / 2},
       {-h / 3},
       {0},
       {1}};
  double pos3[4][1] =
      {{0},
       {-2 * h / 3},
       {0},
       {1}};

  double next_servo_pos()
  {
    double a = multiply(4, 4, P_roti, 4, 1, pos1) 
    (P_roti, pos1);
    double b = MultiplyWithTiling(P_roti, pos2);
    double c = MultiplyWithTiling(P_roti, pos3);
    double servo_heights = {{a[3][1]}, {b[3][1]}, {c[3][1]}};
    double servo_ang[3];
    for (int i = 1; i < size(servo_heights); i++)
    {
      double elbow_theta = M_PI - std::acos((-x * x - servo_heights[i] * servo_heights[i] + L_1 * L_1 + L_2 * L_2) / (2 * L_1 * L_2));
      double servo_v = std::atan(x / servo_heights[i]) - std::atan((L_2 * std::sin(elbow_theta)) / (L_1 + L_2 * ::std::cos(elbow_theta)));
      servo_ang[i] = servo_v;     
    }
    return servo_ang[3];
  }


  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Coordinates recivced: [%f, %f, %f]", msg->data[0], msg->data[1], msg->data[2]);

    double x_think = PID_X.compute(0, msg->data[0]);
    double y_think = PID_Y.compute(0, msg->data[1]);

    float max_pid = 10;
    float min_pid = -10;
    // Map output of PID to table tilt
    // Pitch X
    float pitch = min_pitch + ((max_pitch - min_pitch) / (max_pid - min_pid)) * (x_think - min_pid);
    float roll_deg = min_roll + ((max_roll - min_roll) / (max_pid - min_pid)) * (y_think - min_pid);

    RCLCPP_INFO(this->get_logger(), "PID IS CRAZY: [%f]", y_think);
    RCLCPP_INFO(this->get_logger(), "ROLL IS CRAZY: [%f]", roll_deg);

    //

  }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
