#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>
#include <memory>
#include <chrono>
#include <algorithm>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"

// Constants and geometric parameters
const double L_1 = 4.0; // Servo arm length
const double L_2 = 9.8; // Forearm length
const double L = 22.5; // Base length
const double h = L * std::sqrt(3.0 / 4.0); // Height derived from geometry

// PID parameters and limits
const double max_pid = 10.0, min_pid = -10.0;
const double servo_angle_limit_max = 85.0, servo_angle_limit_min = -35.0;
const double max_pitch = 8.0, min_pitch = -5.0;
const double max_roll = 8.0, min_roll = -5.0;

// Position matrices for transformation
const double pos1_matrix[4] = {-L/2.0, -h/3.0, 0.0, 1.0};
const double pos2_matrix[4] = {L/2.0, -h/3.0, 0.0, 1.0};
const double pos3_matrix[4] = {0.0, 2.0*h/3.0, 0.0, 1.0};

class PID {
    double kp, ki, kd;
    double integral = 0.0, previous_error = 0.0;
    std::chrono::steady_clock::time_point last_time;
    bool integral_limit = true;
    int integral_cap = 20;

public:
    PID(double p, double i, double d) : kp(p), ki(i), kd(d) {
        last_time = std::chrono::steady_clock::now();
    }

    double compute(double setpoint, double ball_pos) {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = now - last_time;
        double delta_time = dt.count(), error = setpoint - ball_pos;
        integral += error * delta_time;

        if (integral_limit) {
            integral = std::clamp(integral, -static_cast<double>(integral_cap), static_cast<double>(integral_cap));
        }

        double derivative = (error - previous_error) / delta_time;
        double output = kp * error + ki * integral + kd * derivative;
        previous_error = error;
        last_time = now;

        return std::clamp(output, min_pid + 1.0, max_pid);
    }
};

std::vector<std::vector<double>> newPos(float gamma_deg, float theta_deg, const double pos[4], float z_) {
    const double rotation_offset_deg = 30.0; // Define the rotation offset
    const double gamma_rad = M_PI/180 * (gamma_deg + rotation_offset_deg); // Apply the rotation offset
    const double theta_rad = M_PI/180 * (theta_deg+ rotation_offset_deg);

    const double rotation_matrix[4][4] = {
        {std::cos(gamma_rad), std::sin(gamma_rad) * std::sin(theta_rad), std::sin(gamma_rad) * std::cos(theta_rad), 0},
        {0.0, std::cos(theta_rad), -std::sin(theta_rad), 0.0},
        {-std::sin(gamma_rad), std::cos(gamma_rad) * std::sin(theta_rad), std::cos(gamma_rad)*std::cos(theta_rad), z_},
        {0.0, 0.0, 0.0, 1.0}
    };

    std::vector<std::vector<double>> newPositionVector = {{0}, {0}, {0}, {0}};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            newPositionVector[i][0] += rotation_matrix[i][j] * pos[j];
        }
    }
    return newPositionVector;
}


std::vector<double> next_servo_pos(float pitch_deg, float roll_deg, float z_) {
    const auto a = newPos(pitch_deg, roll_deg, pos1_matrix, z_);
    const auto b = newPos(pitch_deg, roll_deg, pos2_matrix, z_);
    const auto c = newPos(pitch_deg, roll_deg, pos3_matrix, z_);

    std::vector<double> servo_ang_deg_test;
    for (const double y : {a[2][0], b[2][0], c[2][0]}) {
        double theta2 = std::acos((1.0 + y*y - L_1*L_1 - L_2*L_2) / (2 * L_2 * L_1));
        double theta1 = std::atan2(y, 1.0) - std::atan2(L_2 * std::sin(theta2), L_1 + L_2 * std::cos(theta2));
        servo_ang_deg_test.push_back(theta1 * 180 / M_PI);
    }
    return servo_ang_deg_test;
}

std::vector<double> servo_angle_outputs(std::vector<double> angles) {
    std::vector<double> output;
    for (double angle : angles) {
        output.push_back(std::clamp(angle, servo_angle_limit_min + 1, servo_angle_limit_max - 1));
    }
    return output;
}

class Platform_Regulator : public rclcpp::Node {
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_, publisher_pid_;
    PID PID_X = PID(0.5, 0.2, 0.1), PID_Y = PID(0.5, 0.2, 0.1);

public:
    Platform_Regulator() : Node("regulator") {
        subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "ball_coordinates", 10, std::bind(&Platform_Regulator::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("servo_angle_array", 10);
        publisher_pid_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("pitch_n_roll", 10);
    }

    void topic_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        float pidx_ = PID_X.compute(376, msg->x);
        float pidy_ = PID_Y.compute(256, msg->y);

        float pitch_deg = min_pitch + ((max_pitch - min_pitch) / (max_pid - min_pid)) * (pidx_ - min_pid);
        float roll_deg = min_roll + ((max_roll - min_roll) / (max_pid - min_pid)) * (pidy_ - min_pid);

        auto anglee = servo_angle_outputs(next_servo_pos(pitch_deg, roll_deg, 10));

        auto array_ = std_msgs::msg::Float32MultiArray();
        array_.data = {anglee[0], anglee[1], anglee[2]};
        publisher_->publish(array_);

        auto array_pitch_roll = std_msgs::msg::Float32MultiArray();
        array_pitch_roll.data = {pitch_deg, roll_deg};
        publisher_pid_->publish(array_pitch_roll);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Platform_Regulator>());
    rclcpp::shutdown();
    return 0;
}

