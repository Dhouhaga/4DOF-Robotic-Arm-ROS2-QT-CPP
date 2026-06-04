//
// Created by chiheb on 10/05/2026.
//
#include "pca9685_pi_hw_interface/pca9685_pi_hw_interface.h"
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <thread>

namespace rpi_pca9685_hw_interface {

    namespace {
        constexpr auto kServoWriteStagger = std::chrono::milliseconds(8);
    }


    hardware_interface::CallbackReturn Pca9685PiHwInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("Pca9685PiHwInterface"), "on_init()");
        if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        if (info_.joints.size()==0 or info_.joints.size() >16) {
            RCLCPP_ERROR(rclcpp::get_logger("Pca9685PiHwInterface"), "Error : Incorrect number of joints, %zu found!", info_.joints.size());
            return CallbackReturn::ERROR;
        }

        auto & info {params.hardware_info};
        // Fetch PCA9685 global parameters
        for (const auto &param : info.hardware_parameters) {
            if (param.first == "i2c_bus") {
                i2c_bus_number_ = std::stoi(param.second);
            } else if (param.first == "i2c_address") {
                i2c_address_ = std::stoi(param.second, nullptr, 0);
            } else if (param.first == "pwm_frequency") {
                pwm_frequency_ = std::stoi(param.second);
            }
        }

        // Configure joints
        // Configuration des joints
        joints_.clear();
        for (const auto &joint : info.joints) {
            JointConfigData config;
            config.name = joint.name;

            for (const auto &param : joint.parameters) {
                if (param.first == "channel") {
                    config.channel = std::stoi(param.second);
                } else if (param.first == "min_pulse_us") {
                    config.min_pulse_us = std::stoi(param.second);
                } else if (param.first == "max_pulse_us") {
                    config.max_pulse_us = std::stoi(param.second);
                } else if (param.first == "min_angle_deg") {
                    config.min_angle = std::stoi(param.second);
                } else if (param.first == "max_angle_deg") {
                    config.max_angle = std::stoi(param.second);
                }
            }
            joints_.push_back(config);
        }

        pca9685_driver_=std::make_unique<Pca9685Driver>(i2c_bus_number_, i2c_address_,pwm_frequency_);
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Pca9685PiHwInterface::export_state_interfaces() {
        RCLCPP_DEBUG(rclcpp::get_logger("Pca9685PiHwInterface"), "Export state interfaces");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto & joint : joints_) {
            state_interfaces.emplace_back(joint.name, "position", &joint.data.hw_state);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Pca9685PiHwInterface::export_command_interfaces() {
        RCLCPP_DEBUG(rclcpp::get_logger("Pca9685PiHwInterface"), "Export command interfaces");
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto & joint : joints_) {
            command_interfaces.emplace_back(joint.name, "position", &joint.data.hw_command);
        }
        return command_interfaces;
    }

    hardware_interface::return_type Pca9685PiHwInterface::read(const rclcpp::Time &,const rclcpp::Duration &) {
        for (auto & joint : joints_) {
            joint.data.hw_state=joint.data.hw_command;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Pca9685PiHwInterface::write(const rclcpp::Time&,const rclcpp::Duration&) {
        for (auto & joint : joints_) {
            if (joint.data.hw_command == joint.data.hw_prev_command) {
                continue;
            }

            try {
                const double clamped_angle = std::clamp<double>(joint.data.hw_command, joint.min_angle, joint.max_angle);
                const double pulse_width = angle_to_pulse_width(joint);

                pca9685_driver_->set_pulse_width(joint.channel, pulse_width);
                RCLCPP_INFO(rclcpp::get_logger("Pca9685PiHwInterface"),
                    "Joint %s on channel %d: angle=%.2f deg (clamped=%.2f deg) -> pwm=%.0f",
                    joint.name.c_str(),
                    joint.channel,
                    joint.data.hw_command,
                    clamped_angle,
                    pulse_width);
                joint.data.hw_prev_command = joint.data.hw_command;
                std::this_thread::sleep_for(kServoWriteStagger);
            }
            catch (std::runtime_error &e) {
                RCLCPP_ERROR(rclcpp::get_logger("Pca9685PiHwInterface"),
                    "Error setting PWM for joint %s on channel %d: %s",
                    joint.name.c_str(),
                    joint.channel,
                    e.what());
                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }

    double Pca9685PiHwInterface::angle_to_pulse_width(const JointConfigData&  joint) const {
        double degree {std::clamp<double>(joint.data.hw_command, joint.min_angle, joint.max_angle)};
        double duration_us= ((joint.max_pulse_us - joint.min_pulse_us) * degree / (joint.max_angle-joint.min_angle)) + joint.min_pulse_us;
        return (duration_us/((1.0/pwm_frequency_)*1'000'000))*4096;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rpi_pca9685_hw_interface::Pca9685PiHwInterface, hardware_interface::SystemInterface)