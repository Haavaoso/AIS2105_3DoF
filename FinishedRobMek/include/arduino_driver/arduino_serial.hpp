#ifndef ARDUINO_DRIVE_ARDUINO_SERIAL_HPP
#define ARDUINO_DRIVE_ARDUINO_SERIAL_HPP

#include <sstream>
#include <serial/serial.h>
#include <iostream>
#include <string>

class ArduinoSerial
{
public:
    ArduinoSerial() = default;

    ArduinoSerial(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    { }

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        serial_conn_.setPort(serial_device);
        serial_conn_.setBaudrate(baud_rate);
        serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
        serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
        serial_conn_.open();
    }

    bool connected() const
    {
        return serial_conn_.isOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_conn_.write(msg_to_send);
        std::string response = serial_conn_.readline();

        if (print_output)
        {
            std::cout << "Sent2: " << msg_to_send << " Recv: " << response << std::endl;
        }

        return response;
    }

    void send_empty_msg()
    {
        std::string response = send_msg("\n");
    }

    void read_motor_values(int &motorA, int &motorB, int &motorC)
    {
        std::string response = send_msg("\n",true);
        std::istringstream iss(response);
        std::string header, token;
        if (std::getline(iss, header, ':'))
        {
            if (header == "MOTORS")
            {
                std::getline(iss, token, ',');
                motorA = std::stoi(token);
                std::cout << "String: " << token << " TransA: " << motorA << std::endl;
                std::getline(iss, token, ',');
                motorB = std::stoi(token);
                std::cout << "String: " << token << " TransB: " << motorB << std::endl;
                std::getline(iss, token, ',');
                motorC = std::stoi(token);
                std::cout << "String: " << token << " TransC: " << motorC << std::endl;
            }
            else
            {
                std::cerr << "Error: Invalid message header." << response << std::endl;
            }
        }
        else
        {
            std::cerr << "Error: Missing message header." << response << std::endl;
        }
    }

    void set_motor_values(int motorA, int motorB, int motorC)
    {
        std::stringstream ss;
        ss << "MOTORS:" << motorA << "," << motorB << "," << motorC << "\n";
        send_msg(ss.str(),true);
        std::cout << "Sent message: " << ss.str() << std::endl;
    }

private:
    serial::Serial serial_conn_;
};

#endif // ARDUINO_DRIVE_ARDUINO_SERIAL_HPP