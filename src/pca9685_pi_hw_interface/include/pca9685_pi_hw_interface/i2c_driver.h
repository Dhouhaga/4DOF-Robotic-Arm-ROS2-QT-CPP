//
// Created by chiheb on 10/05/2026.
//

#ifndef WS_ROS_PI_I2C_DRIVER_H
#define WS_ROS_PI_I2C_DRIVER_H
#include <cstdint>

namespace rpi_pca9685_hw_interface {

    struct I2cDriver {
        I2cDriver(uint8_t i2c_bus, uint8_t i2c_address);

        void write_byte(uint8_t reg, uint8_t value);

        uint8_t read_byte(uint8_t reg);

        I2cDriver& set_slave();

        I2cDriver& set_7bits_mode();
        ~I2cDriver();
    private:
        int file_descriptor_;
        uint8_t i2c_address_;
        enum class I2cOptions {
            I2C_TENBIT = 0x0704,
            I2C_SLAVE = 0x0706
        };
    };
}

#endif //WS_ROS_PI_I2C_DRIVER_H
