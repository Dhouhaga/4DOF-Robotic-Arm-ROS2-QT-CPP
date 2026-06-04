#ifndef RPI_PCA9685_HW_CONTROLLER_H
#define RPI_PCA9685_HW_CONTROLLER_H
#include "pca9685_pi_hw_interface/pca9685_driver.h"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rpi_pca9685_hw_interface {
    struct Pca9685PiHwInterface : public hardware_interface::SystemInterface {
        Pca9685PiHwInterface() = default;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    private:
        struct JointConfigData {
            std::string name;
            uint8_t channel;
            uint8_t min_angle {};
            uint8_t max_angle {180};
            uint16_t min_pulse_us {1000};
            uint16_t max_pulse_us {2000};
            struct data {
                double hw_command {};
                double hw_prev_command {};
                double hw_state {};
            } data;
        };
        uint8_t i2c_bus_number_ {1};
        uint8_t i2c_address_ {0x40};
        uint8_t pwm_frequency_ {50};
        std::vector<JointConfigData> joints_;
        std::unique_ptr<Pca9685Driver> pca9685_driver_;
        double angle_to_pulse_width(const JointConfigData&  joint) const;
    };
}
#endif
