
Architecture Overview
---------------------
This package provides a ROS2 `hardware_interface::SystemInterface` implementation for Raspberry Pi to control a PCA9685 16-channel PWM driver over I2C.

Core components:
- `Pca9685PiHwInterface` (ros2_control SystemInterface): exports position command/state interfaces per joint, reads hardware and joint parameters, performs angle→pulse conversion, and implements the read/write lifecycle.
- `Pca9685Driver`: low-level PCA9685 register access and utilities (initialize, set PWM frequency, write LED registers, sleep/wake, helpers such as stop_channel and set_all_pulse_width).
- `I2cDriver`: minimal Linux I2C wrapper using /dev/i2c-N with ioctl for slave addressing and 7-bit mode.

Design notes: the package separates responsibilities — conversion logic lives in the hardware interface, while the driver only performs register/pulse writes. Default PWM frequency is 50 Hz and up to 16 channels are supported.

Build & development
-------------------
- Dependencies: ROS2 Jazzy packages (`hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`).
- Build: standard ament_cmake.
- Plugin export: the package exports `pca9685_pi_hw_interface.xml` via CMake so `ros2_control_node` can load the plugin at runtime.

Key integrations
----------------
- ROS2 ros2_control: hardware is configured from the URDF/xacro `ros2_control` block or from `controllers.yaml`.
- Hardware: PCA9685 at I2C address 0x40 by default; Raspberry Pi I2C bus typically `/dev/i2c-1`.

Code patterns & conventions
-------------------------
- Class responsibilities:
  - `I2cDriver`: open/close bus, read/write bytes, throws std::runtime_error on failures.
  - `Pca9685Driver`: register-level helpers and bulk operations; does not map angles to pulse widths.
  - `Pca9685PiHwInterface`: reads parameters in `on_init`, fills `JointConfigData`, exports interfaces, implements `read` and `write`, and contains `angle_to_pulse_width(const JointConfigData&)`.
- Parameter expectations: for each joint, provide `channel`, `min_pulse_us`, `max_pulse_us`, `min_angle_deg`, and `max_angle_deg` in the URDF `ros2_control` joint parameters.
- Read/write behavior: `read` copies commands to states (no sensor feedback). `write` converts commanded position to pulse width then calls `Pca9685Driver::set_pulse_width`.
- Error handling: I2C failures propagate as exceptions; the hardware interface catches runtime errors and returns `hardware_interface::return_type::ERROR` from `write`.

Important files
---------------
- `include/pca9685_pi_hw_interface/pca9685_pi_hw_interface.h` — `Pca9685PiHwInterface` and `JointConfigData` definition.
- `src/pca9685_pi_hw_interface.cpp` — ROS2 hardware interface implementation.
- `include/pca9685_pi_hw_interface/pca9685_driver.h` and `src/pca9685_driver.cpp` — PCA9685 helpers.
- `include/pca9685_pi_hw_interface/i2c_driver.h` and `src/i2c_driver.cpp` — Linux I2C wrapper.
- `pca9685_pi_hw_interface.xml` — pluginlib description installed to share directory.

