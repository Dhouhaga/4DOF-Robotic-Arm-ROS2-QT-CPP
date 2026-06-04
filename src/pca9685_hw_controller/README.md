# Demo: PCA9685 4-servo robot

This demo shows how to run a simple 4-servo robot using the `pca9685_pi_hw_interface` hardware plugin and the `forward_position_controller`.

Prerequisites
--------------
- A built workspace (source your workspace `install/local_setup.bash`).
- On the Raspberry Pi target: I2C enabled (e.g. via `raspi-config`) and permissions to access `/dev/i2c-1`.
- `xacro` available on the machine that launches the demo (or a pre-generated URDF placed in `urdf/robot_with_pca9685.urdf`).

Quick start
-----------
1. In one terminal (on the Raspberry Pi) run the launch:

    ros2 launch pca9685_hw_controller robot_4servo.launch.py

This launch will:
- publish the `robot_description` (generated from `urdf/robot_with_pca9685.urdf.xacro` or using the pre-generated URDF),
- start `robot_state_publisher`,
- start `ros2_control_node` with the controllers defined in `config/controllers.yaml`,
- spawn the `joint_state_broadcaster` and `forward_position_controller`.

2. In another terminal you can send a single position command to the `forward_position_controller`:

    ros2 topic pub -1 /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [170, 0.0, 0.0, 0.0]"

Notes
-----
- The array elements correspond to joints in this order: `servo_pan`, `servo_tilt`, `gripper`, `wrist`.
- Values are in degrees and will be clamped to each joint's `min_angle_deg`/`max_angle_deg` range defined in the URDF.
- If you see errors about missing `robot_description` or invalid URDF, ensure `xacro` is installed or generate the URDF manually. Example command to render URDF from xacro:

    xacro src/pca9685_hw_controller/urdf/robot_with_pca9685.urdf.xacro -o src/pca9685_hw_controller/urdf/robot_with_pca9685.urdf

Developer tips
--------------
- Check `config/controllers.yaml` to edit controller names or update rates.
- The hardware plugin name used in the URDF is `rpi_pca9685_hw_interface/Pca9685PiHwInterface`.

Files of interest
-----------------
- `launch/robot_4servo.launch.py` — launch file used by the demo.
- `config/controllers.yaml` — controller and hardware parameters (including `PCA9685System` hardware parameters).
- `urdf/robot_with_pca9685.urdf.xacro` — robot description template.


