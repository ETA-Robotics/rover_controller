#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Joy

class RoverJoyTranslator(Node):
    def __init__(self):
        super().__init__('rover_joy_translator')

        self.get_logger().info(f"Running Rover_joy translator Node")

        # Setup Parameters
        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")

        # Subscriptions
        self.joy1_subscription = self.create_subscription(
            Joy,
            '/joy1',
            self.joy1_callback,
            10)

        self.joy2_subscription = self.create_subscription(
            Joy,
            '/joy2',
            self.joy2_callback,
            10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Float32, '/cmd_vel', 10)
        self.cmd_stop_publisher = self.create_publisher(Bool, '/cmd_stop', 10)
        self.cmd_cal_publisher = self.create_publisher(Bool, '/cmd_cal', 10)
        self.cmd_steer_publisher = self.create_publisher(Float32, '/cmd_steer', 10)

    def joy1_callback(self, msg):
        # X52 Controller (HOTAS)
        # Extract velocity command
        cmd_vel = Float32()
        cmd_vel.data = msg.axes[2] # Index 2 is velocity
        self.cmd_vel_publisher.publish(cmd_vel)

        # Extract stop command
        cmd_stop = Bool()
        cmd_stop.data = bool(msg.buttons[7])  # Button 7 is the stop command
        self.cmd_stop_publisher.publish(cmd_stop)

        # Extract calibration command
        cmd_cal = Bool()
        cmd_cal.data = bool(msg.buttons[30])  # Button 30 is the calibration command
        self.cmd_cal_publisher.publish(cmd_cal)

        # Logging
        if (self.debug_serial_cmds):
            self.get_logger().info(f"Published cmd_vel: {cmd_vel.data}, cmd_stop: {cmd_stop.data}, cmd_cal: {cmd_cal.data}")

    def joy2_callback(self, msg):
        # Yoke
        # Extract angle command for steering
        cmd_steer = Float32()
        cmd_steer.data = msg.axes[0]  # Index 0 is steering angle
        self.cmd_steer_publisher.publish(cmd_steer)

        # Logging
        if (self.debug_serial_cmds):
            self.get_logger().info(f"Published cmd_steer: {cmd_steer.data}")


def main(args=None):
    rclpy.init(args=args)
    node = RoverJoyTranslator()
    rclpy.spin(node)
    rclpy.shutdown()
