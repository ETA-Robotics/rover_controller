#!/usr/bin/env python3
import rclpy
import serial
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from threading import Lock
# try:
#     ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
#     ser.reset_input_buffer()
# except serial.SerialException as e:
#     print(f"Error opening or using serial port: {e}")
# except Exception as e:
#     print(f"An unexpected error occurred: {e}")

## Combine Vel and Steeting channel into single object
## i.e. {cmd_vel, cmd_ster, cmd_brake, cmd_cali}

class JoystickSubscriber(Node):

    def __init__(self):
        super().__init__('joystick_subscriber')


        # Setup Parameters

        self.declare_parameter('serial_port', value="/dev/ttyUSB0")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=57600)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('serial_debug', value=True)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")


        # Setup topics & services

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

        # self.get_logger().info("Joystick Subscriber has been started 3.1")


        # Member Variables

        #self.mutex = Lock()


        # Open Serial Comms

        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")



    def joy1_callback(self, msg):
        # X52 controller (HOTAS)
        throttle = msg.axes[2]
        throttle = round(throttle, 3)
        stopButton = msg.buttons[7]
        CalibrationButton = msg.buttons[30]
 
        if(stopButton == 1):     
            data = f"{'2.0'}" 
        elif (CalibrationButton == 1):
            data = f"{'4.0'}"
        else :
            data = f"{throttle}"
        
        self.send_command('B', data)  # Packet type B


    def joy2_callback(self, msg):
        # Yoke
        angle = msg.axes[0]
        angle = round(angle, 3)
        data = angle

        self.send_command('C', data)  # Packet type C
        

    # def send_packet(self, packet_id, data):
    #     message = f"{packet_id},{data}\n"  
    #     ser.write(message.encode())

    
    # Utility functions

    def send_command(self, cmd, data):
        """
        Sends a command with associated data to the connection.
        
        Args:
            cmd (str): The command to send, e.g., "C".
            data (float or str): The data to send with the command, e.g., 4.0.
        """        
        #self.mutex.acquire()
        try:
            cmd_string = f"{cmd},{data}"  # Format the command string
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))  # Send as UTF-8 encoded bytes
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)


            ## Adapted from original
            # c = ''
            # value = ''
            # while c != '\r':
            #     c = self.conn.read(1).decode("utf-8")
            #     if (c == ''):
            #         print("Error: Serial timeout on command: " + cmd_string)
            #         return ''
            #     value += c

            # value = value.strip('\r')

            # if (self.debug_serial_cmds):
            #     print("Received: " + value)
            # return value
        finally:
            time.sleep(0.05)
            return
            #self.mutex.release()

       

    def close_conn(self):
        self.conn.close()


def main(args=None):
    # rclpy.init(args=args)
    # node = JoystickSubscriber()
    # rclpy.spin(node)
    # rclpy.shutdown()

    rclpy.init(args=args)

    joystick_subscriber = JoystickSubscriber()

    while rclpy.ok():
        rclpy.spin_once(joystick_subscriber)

    joystick_subscriber.close_conn()
    joystick_subscriber.destroy_node()

    rclpy.shutdown()