#!/usr/bin/env python3
"""
joint_state_bridge.py
---------------------
Subscribes to  /arm_viz/commands
  (std_msgs/Float64MultiArray  [base_deg, shoulder_deg, wrist_deg, gripper_deg])
Publishes to   /joint_states
  (sensor_msgs/JointState)

Joint mapping (matches the URDF):
  index 0 → waist       (Z rotation,  deg → rad)
  index 1 → shoulder    (Y rotation,  deg → rad)
  index 2 → elbow       (Y rotation,  deg → rad)
  index 3 → gripper     (finger open/close, deg → rad, clamped to [-0.5, 0.0])

Angles arriving from the dashboard are 0-180 deg.
We map them so that 90° == 0 rad (neutral / centre position).
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


# How many degrees offset from 90° maps to 0 rad (neutral)
NEUTRAL_DEG = 90.0


def deg_to_rad(deg: float) -> float:
    return math.radians(deg - NEUTRAL_DEG)


def gripper_deg_to_rad(deg: float) -> float:
    """
    Gripper servo 0-180 deg → finger joint -0.5 to 0.0 rad.
    90 deg = fully open (0.0 rad limit), 180 deg = fully closed (-0.5 rad).
    Clamp to joint limits defined in URDF.
    """
    # Map [0, 180] → [0.0, -0.5]  (invert: higher angle = more closed)
    rad = -0.5 * (deg / 180.0)
    return max(-0.5, min(0.0, rad))


class JointStateBridge(Node):

    def __init__(self):
        super().__init__('joint_state_bridge')

        self.joint_names = ['waist', 'shoulder', 'elbow', 'gripper']

        # Start at neutral (90 deg for all joints)
        self.positions = [0.0, 0.0, 0.0, 0.0]

        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/arm_viz/commands',
            self.on_command,
            10
        )

        # Publish at 30 Hz even when no new command arrives
        # so RViz always has a fresh transform
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)

        self.get_logger().info('joint_state_bridge ready — waiting for /arm_viz/commands...')

    def on_command(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) < 4:
            self.get_logger().warn(f'Expected 4 values, got {len(data)}')
            return

        self.positions[0] = deg_to_rad(data[0])       # waist
        self.positions[1] = deg_to_rad(data[1])       # shoulder
        self.positions[2] = deg_to_rad(data[2])       # elbow
        self.positions[3] = gripper_deg_to_rad(data[3])  # gripper

        self.get_logger().info(
            f'CMD  base={data[0]:.1f}° sh={data[1]:.1f}° '
            f'wr={data[2]:.1f}° gr={data[3]:.1f}°  →  '
            f'rad [{self.positions[0]:.3f}, {self.positions[1]:.3f}, '
            f'{self.positions[2]:.3f}, {self.positions[3]:.3f}]'
        )

    def publish_joint_states(self):
        msg = JointState()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.name = self.joint_names
        msg.position = self.positions
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
