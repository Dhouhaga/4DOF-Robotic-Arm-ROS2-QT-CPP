//
// Created by chiheb on 10/05/2026.
//

#include "pca9685_pi_hw_interface/i2c_driver.h"
#include <cerrno>
#include <cstring>
#include <string>
#include <fcntl.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>
namespace rpi_pca9685_hw_interface {
    I2cDriver::I2cDriver(uint8_t i2c_bus, uint8_t i2c_address) : i2c_address_(i2c_address) {
        // Open the I2C bus

        std::string bus_path = "/dev/i2c-" + std::to_string(i2c_bus);
        file_descriptor_ = open(bus_path.c_str(), O_RDWR);
        if (file_descriptor_ < 0) {
            throw std::runtime_error("Failed to open I2C bus " + bus_path + ": " + std::strerror(errno));
        }
    }

    I2cDriver& I2cDriver::set_slave() {
        // Set the I2C slave address
        if (ioctl(file_descriptor_, static_cast<long unsigned int>(I2cOptions::I2C_SLAVE), i2c_address_) < 0) {
            throw std::runtime_error("Failed to set I2C slave address 0x" + std::to_string(i2c_address_) + ": " + std::strerror(errno));
        }
        return *this;
    }

    I2cDriver& I2cDriver::set_7bits_mode() {
        if  (ioctl(file_descriptor_, static_cast<long unsigned int>(I2cOptions::I2C_TENBIT), 0)<0) {
            throw std::runtime_error("Failed to set I2C to 7-bit mode: " + std::string(std::strerror(errno)));
        }
        return *this;
    }


    void I2cDriver::write_byte(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] {reg, value};
        if (write(file_descriptor_, buffer, 2) != 2) {
            throw std::runtime_error("Failed to write byte to I2C device: " + std::string(std::strerror(errno)));
        }
    }

    uint8_t I2cDriver::read_byte(uint8_t reg) {
        if (write(file_descriptor_, &reg, 1) != 1) {
            throw std::runtime_error("Failed to write register address to I2C device: " + std::string(std::strerror(errno)));
        }
        uint8_t value;
        if (read(file_descriptor_, &value, 1) != 1) {
            throw std::runtime_error("Failed to read byte from I2C device: " + std::string(std::strerror(errno)));
        }
        return value;
    }

    I2cDriver::~I2cDriver() {
        if (file_descriptor_ >= 0) {
            close(file_descriptor_);
        }
    }
}