//
// Created by chiheb on 11/05/2026.
//

#include "pca9685_pi_hw_interface/pca9685_driver.h"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
namespace rpi_pca9685_hw_interface {
    Pca9685Driver::Pca9685Driver(uint8_t i2c_bus, uint8_t i2c_address,uint8_t freq):i2c_driver_(std::make_unique<I2cDriver>(i2c_bus,
        i2c_address)) {

        (*i2c_driver_).set_slave().set_7bits_mode();
        // Initialize the PCA9685
        i2c_driver_->write_byte(PCA9685_MODE1, 0x00); // MODE1 register: normal mode
        i2c_driver_->write_byte(0x01, 0x04); // MODE2 register: totem pole output
        set_pwm_frequency(freq);
    }

    void Pca9685Driver::set_pwm_frequency(uint8_t freq) {
        float frequency{freq*0.8449f};
        float prescaleval {25'000'000};       // 25MHz
        prescaleval /= 4096;                // 4096 PWM cycles
        prescaleval /= frequency;                // Calculate the frequency division value
        prescaleval -= 1;
        float prescale {prescaleval + 0.5f};
        uint8_t oldmode {i2c_driver_->read_byte(PCA9685_MODE1)}; // MODE1 register
        uint8_t newmode {static_cast<uint8_t>((oldmode&0x7F) | 0x10)};
        i2c_driver_->write_byte(PCA9685_MODE1, newmode); // Go to sleep
        sleep();
        i2c_driver_->write_byte(PCA9685_PRESCALE, static_cast<uint8_t>(prescale)); // Set predivider
        i2c_driver_->write_byte(PCA9685_MODE1, oldmode);
        std::this_thread::sleep_for(5000us);
        i2c_driver_->write_byte(PCA9685_MODE1, oldmode | 0xa0);
        current_frequency_ = freq; // Mettre à jour la fréquence actuelle
    }


    void Pca9685Driver::set_pulse_width(uint8_t channel,uint16_t pw) {
        i2c_driver_->write_byte(LED0_ON_L+4*channel, 0);
        i2c_driver_->write_byte(LED0_ON_L+4*channel + 1,0);
        i2c_driver_->write_byte(LED0_ON_L+4*channel + 2, pw & 0xFF);
        i2c_driver_->write_byte(LED0_ON_L+4*channel + 3, pw>>8);
    }

    void Pca9685Driver::set_all_pulse_width(uint16_t pw) {
        i2c_driver_->write_byte(ALL_LED_ON_L, 0);
        i2c_driver_->write_byte(ALL_LED_ON_H, 0);
        i2c_driver_->write_byte(ALL_LED_OFF_L, pw & 0xFF);
        i2c_driver_->write_byte(ALL_LED_OFF_H, pw >> 8);
    }



    void Pca9685Driver::stop_channel(uint8_t channel) {
        set_pulse_width(channel, 0);
    }

    void Pca9685Driver::stop_all_channels() {
        set_all_pulse_width(0);
    }

    void Pca9685Driver::set_channel_brightness(uint8_t channel, uint8_t brightness) {
        // Convertit la luminosité 0-255 en pulse width 0-4095
        uint16_t pulse_width = static_cast<uint16_t>((brightness * 4095UL) / 255);
        set_pulse_width(channel, pulse_width);
    }

    void Pca9685Driver::sleep() {
        uint8_t mode1 {i2c_driver_->read_byte(PCA9685_MODE1)};
        mode1 |= 0x10; // Set sleep bit
        i2c_driver_->write_byte(PCA9685_MODE1, mode1);
        is_sleeping_ = true;
    }

    void Pca9685Driver::wake() {
        uint8_t mode1 = i2c_driver_->read_byte(PCA9685_MODE1);
        mode1 &= ~0x10; // Clear sleep bit
        i2c_driver_->write_byte(PCA9685_MODE1, mode1);
        std::this_thread::sleep_for(500us); // Wait for oscillator to stabilize
        mode1 |= 0xA0; // Set restart and auto-increment bits
        i2c_driver_->write_byte(PCA9685_MODE1, mode1);
        is_sleeping_ = false;
    }

    uint8_t Pca9685Driver::get_current_frequency() const {
        return current_frequency_;
    }

    bool Pca9685Driver::is_sleeping() const {
        return is_sleeping_;
    }

}