#include <vector>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <vector>
#include <iostream>


double pi = M_PI;

double theta_deg = 0.0; 
double theta = theta_deg*pi/180.0; //pitch frå subscriber

double gamma_deg = 0.0;
double gamma1 = gamma_deg * pi / 180.0; //roll frå subscriber, heite gamma1 fordi nokke navnkollisjon, kan sikk endrast.

double alpha_deg = 150.0;
double alpha = alpha_deg*pi/180.0;
double z = 45.0; //lengde til ball

double x = 300.00; //piksel x koordinat
double y = 200.0; //piksel y koordinat



std::vector<std::vector<double>> cameraMatrix ={ // DITTA E INVERSE KAMERAMATRISE
        {0.001574803149606, 0.0, -0.544881889763779},
        {0.0, 0.001572327044025, -0.355345911949686},
        {0.0, 0.0, 1.0}
};

std::vector<std::vector<double>> getGaming(){
    auto a = matrixMultiplication(transformationMatrix,zMatrix);
    auto b = matrixMultiplication(a,cameraMatrix);
    auto c = matrixMultiplication(b, pixelPos);
    return c;
    }





class Kinematics_position : public rclcpp::Node
{
public:
  Kinematics_position()
      : Node("camera_kin")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
        "ball_coordinates", 10, std::bind(&Kinematics_position::position_callback, this, std::placeholders::_1));

    subscription_pid = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "pitch_n_roll", 10, std::bind(&Kinematics_position::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("Ball_position_XYZ", 10);
  }

private:
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg, const std_msgs::msg::Float32MultiArray::SharedPtr pitch_n_roll)
    {
        //Fra mariannes node
        x = msg->x;
        y = msg->y;
        z = msg->z;

        auto gamma = pitch_n_roll[0];
        auto theta = pitch_n_roll[1];

        std::vector<std::vector<double>> pixelPos ={
        {x},
        {y},
        {1.0}
        };



std::vector<std::vector<double>> transformationMatrix = { // TRANSFORMATION FRÅ KAMERA TIL WORLD KOORDINATSYSTEM
        {cos(gamma1) * cos(alpha), sin(gamma1) * sin(theta) * cos(alpha) - sin(alpha) * cos(theta), cos(alpha) * sin(gamma1) * cos(theta) + sin(alpha) * sin(theta), 0.0},
        {sin(alpha)*cos(gamma1),   sin(gamma1) * sin(theta) * sin(alpha) + cos(alpha) * cos(theta), sin(alpha) * sin(gamma1) * cos(theta) - sin(theta) * cos(alpha), 0.0},
        {-sin(gamma1),             cos(gamma1) * sin(theta),                                        cos(gamma1) * cos(theta),                                        0.0},
        {0.0,                        0.0,                                                               0.0,                                                               1.0}
};

std::vector<std::vector<double>> zMatrix ={
        {z, 0.0, 0.0},
        {0.0, z, 0.0},
        {0.0, 0.0, z},
        {0.0, 0.0, 1.0},
};

std::vector<std::vector<double>> matrixMultiplication(std::vector<std::vector<double>> mat1, std::vector<std::vector<double>> mat2) {
    auto m = mat1.size(); 
    auto n = mat1[0].size();
    auto p = mat2[0].size();
    std::vector<std::vector<double>> result(m, std::vector<double>(p, 0));
    for (int i = 0; i < m; i++) {
        for (int j = 0.0; j < p; j++) {
            for (int k = 0; k < n; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    return result;
}

    auto imsdal_vannflaske = getGaming();
      
    x_ = imsdal_vannflaske[0];
    y_ = imsdal_vannflaske[1];
    z_ = imsdal_vannflaske[2];
      
    auto array_ = std_msgs::msg::Float32MultiArray();
    array_.data = {x_, y_, z_};
  
    publisher_->publish(array_);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_pid;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    float x;
    float y;
    float z;
    float x_;
    float y_;
    float z_;
    float gamma;
    float theta;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Kinematics_position>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
