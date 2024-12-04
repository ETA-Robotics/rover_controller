#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class JoystickSubscriber(Node):

    def __init__(self):
        super().__init__('joystick_subscriber')

        # Setup Parameters
        self.declare_parameter('serial_port', value="/dev/ttyUSB0")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=57600)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if self.debug_serial_cmds:
            print("Serial debug enabled")

        # Setup Serial connection
        try:
            self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)  # 1s timeout for reading
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial connection: {e}")
            raise

        # Subscribers for each topic
        self.cmd_vel_subscription = self.create_subscription(
            Float32,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.cmd_stop_subscription = self.create_subscription(
            Bool,
            '/cmd_stop',
            self.cmd_stop_callback,
            10)

        self.cmd_cal_subscription = self.create_subscription(
            Bool,
            '/cmd_cal',
            self.cmd_cal_callback,
            10)

        self.cmd_steer_subscription = self.create_subscription(
            Float32,
            '/cmd_steer',
            self.cmd_steer_callback,
            10)

        # Initialize all the command variables
        self.cmd_vel = 0.0
        self.cmd_steer = 0.0
        self.cmd_stop = False
        self.cmd_cal = False

        # Track last received time
        self.last_received_time = self.get_clock().now()

        # Create a timer that fires every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(1, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg.data
        if (self.debug_serial_cmds):
            self.get_logger().info(f"Received cmd_vel: {self.cmd_vel}")
        self.last_received_time = self.get_clock().now()

    def cmd_stop_callback(self, msg):
        self.cmd_stop = msg.data
        if (self.debug_serial_cmds):
            self.get_logger().info(f"Received cmd_stop: {self.cmd_stop}")
        self.last_received_time = self.get_clock().now()

    def cmd_cal_callback(self, msg):
        self.cmd_cal = msg.data
        if (self.debug_serial_cmds):
            self.get_logger().info(f"Received cmd_cal: {self.cmd_cal}")
        self.last_received_time = self.get_clock().now()

    def cmd_steer_callback(self, msg):
        self.cmd_steer = msg.data
        if (self.debug_serial_cmds):
            self.get_logger().info(f"Received cmd_steer: {self.cmd_steer}")
        self.last_received_time = self.get_clock().now()

    def send_command(self):
        # Create a single message string containing all command values
        cmd_string = f"{self.cmd_vel},{self.cmd_steer},{self.cmd_stop},{self.cmd_cal}\r"
        
        try:
            # Send the combined message over serial
            self.conn.write(cmd_string.encode("utf-8"))
            # if self.debug_serial_cmds:
            self.get_logger().info(f"Sent: {cmd_string}")

            # Wait for acknowledgment (max 1 second)
            ack_msg = self.receive_acknowledgment()

            if ack_msg:
                self.parse_acknowledgment(ack_msg)

        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")

    def receive_acknowledgment(self):
        """Wait for acknowledgment message from the serial device."""
        start_time = time.time()
        while time.time() - start_time < 1:  # Wait for 1 second max
            if self.conn.in_waiting > 0:
                ack_data = self.conn.readline().decode("utf-8").strip()
                if self.debug_serial_cmds:
                    self.get_logger().info(f"Received acknowledgment: {ack_data}")
                return ack_data
        self.get_logger().error("Acknowledgment timeout.")
        return None

    def parse_acknowledgment(self, ack_msg):
        """Parse acknowledgment message and extract values."""
        try:
            # Example format: STATE,steer1,steer2,motor1_vel,motor2_vel,motor3_vel,motor4_vel
            parts = ack_msg.split(',')
            if len(parts) == 7:
                state = parts[0]  # The robot's state
                steer1 = float(parts[1])
                steer2 = float(parts[2])
                motor1_vel = str(parts[3])
                motor2_vel = str(parts[4])
                motor3_vel = float(parts[5])
                motor4_vel = float(parts[6])

                # Log the extracted values
                #if (self.debug_serial_cmds):
                self.get_logger().info(f"State: {state}, Steer1: {steer1}, Steer2: {steer2}, Motor Velocities: {motor1_vel}, {motor2_vel}, {motor3_vel}, {motor4_vel}")
                
                # Handle robot's state and update accordingly
                if (self.debug_serial_cmds):
                    if state == "IDLE":
                        self.get_logger().info("Robot is idle.")
                    elif state == "MOVING":
                        self.get_logger().info("Robot is moving.")
                    elif state == "CALIBRATING":
                        self.get_logger().info("Robot is calibrating.")
                    else:
                        self.get_logger().warning(f"Unknown state: {state}")
                
                # You can now use these values to update internal states, publish them, etc.
            else:
                self.get_logger().error(f"Invalid acknowledgment format: {ack_msg}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse acknowledgment: {e}")

    def timer_callback(self):
        # Check if we received an update within the last second
        current_time = self.get_clock().now()
        time_since_last_received = (current_time - self.last_received_time).nanoseconds / 1e9


        if time_since_last_received < 1:
            # Send the command message every 0.1s if we received new data within the last second
            self.send_command()
        else:
            # Log that no update has been received in the last second
            if self.debug_serial_cmds:
                self.get_logger().info("No updates received in the last second, skipping serial transmission.")

    def close_conn(self):
        self.conn.close()

def main(args=None):
    rclpy.init(args=args)
    joystick_subscriber = JoystickSubscriber()

    while rclpy.ok():
        rclpy.spin_once(joystick_subscriber)

    joystick_subscriber.close_conn()
    joystick_subscriber.destroy_node()
    rclpy.shutdown()
