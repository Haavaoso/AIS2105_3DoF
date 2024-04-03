// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arduino_driver/arduino_driver.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <tuple>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arduino_driver
{
    hardware_interface::CallbackReturn ThreeDOFArduinoHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        cfg_.motor_A_name = info_.hardware_parameters["motor_A_name"];
        cfg_.motor_B_name = info_.hardware_parameters["motor_B_name"];
        cfg_.motor_C_name = info_.hardware_parameters["motor_C_name"];
        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

        motorA.setup(cfg_.motor_A_name);
        motorB.setup(cfg_.motor_B_name);
        motorC.setup(cfg_.motor_C_name);

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ThreeDOFArduinoHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ThreeDOFArduinoHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ThreeDOFArduinoHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ThreeDOFArduinoHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ThreeDOFArduinoHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ThreeDOFArduinoHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
                motorA.name, hardware_interface::HW_IF_POSITION, &motorA.angle));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            motorA.name, hardware_interface::HW_IF_VELOCITY, &motorA.angular_velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
                motorB.name, hardware_interface::HW_IF_POSITION, &motorB.angle));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            motorB.name, hardware_interface::HW_IF_VELOCITY, &motorB.angular_velocity));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
                motorC.name, hardware_interface::HW_IF_POSITION, &motorC.angle));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            motorC.name, hardware_interface::HW_IF_VELOCITY, &motorC.angular_velocity));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ThreeDOFArduinoHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            motorA.name, hardware_interface::HW_IF_POSITION, &motorA.angle_cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            motorB.name, hardware_interface::HW_IF_POSITION, &motorB.angle_cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            motorC.name, hardware_interface::HW_IF_POSITION, &motorC.angle_cmd));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ThreeDOFArduinoHardware::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeDOFArduinoHardware"), "Configuring ...please wait...");
        comms_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
        RCLCPP_INFO(rclcpp::get_logger("ThreeDOFArduinoHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeDOFArduinoHardware::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeDOFArduinoHardware"), "Cleaning up ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("ThreeDOFArduinoHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeDOFArduinoHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeDOFArduinoHardware"), "Activating ...please wait...");
        if (!comms_.connected())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("ThreeDOFArduinoHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ThreeDOFArduinoHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("ThreeDOFArduinoHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("ThreeDOFArduinoHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ThreeDOFArduinoHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        int motorA_int, motorB_int, motorC_int;

        comms_.read_motor_values(motorA_int, motorB_int, motorC_int);

        double delta_seconds = period.seconds();
        double angle_prev = 0;

        angle_prev = motorA.angle;
        motorA.angle = static_cast<double>(motorA_int);
        motorA.angular_velocity = (motorA.angle - angle_prev) / delta_seconds;

        angle_prev = motorB.angle;
        motorB.angle = static_cast<double>(motorB_int);
        motorB.angular_velocity = (motorB.angle - angle_prev) / delta_seconds;

        angle_prev = motorC.angle;
        motorC.angle = static_cast<double>(motorC_int);
        motorC.angular_velocity = (motorC.angle - angle_prev) / delta_seconds;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type arduino_driver ::ThreeDOFArduinoHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        comms_.set_motor_values(motorA.angle_cmd, motorB.angle_cmd, motorC.angle_cmd);
        return hardware_interface::return_type::OK;
    }

} // namespace arduino_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    arduino_driver::ThreeDOFArduinoHardware, hardware_interface::SystemInterface)