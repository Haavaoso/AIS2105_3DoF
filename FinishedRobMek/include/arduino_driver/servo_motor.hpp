#ifndef ARDUINO_DRIVER_SERVO_MOTOR_HPP
#define ARDUINO_DRIVER_SERVO_MOTOR_HPP

#include <string>
#include <cmath>

class ServoMotor
{
public:
    std::string name = "";
    double angle_cmd = 0;
    double angle = 0;
    double angular_velocity = 0;

    ServoMotor() = default;

    ServoMotor(const std::string &motor_name)
    {
        setup(motor_name);
    }

    void setup(const std::string &motor_name)
    {
        name = motor_name;
    }
};

#endif // ARDUINO_DRIVER_SERVO_MOTOR_HPP