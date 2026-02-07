import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import serial
import struct
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.ser = None
        
        # --- AUTO-CONNECT LOGIC ---
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        for port in ports_to_try:
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.get_logger().info(f"✅ Connected to ESP32 on {port}")
                break # Stop searching if we found it
            except:
                continue # Try the next one
        
        if self.ser is None:
            self.get_logger().error("❌ ESP32 Not Found! Checked: USB0, USB1, ACM0")

        self.sub = self.create_subscription(Int16MultiArray, '/rov/thruster_pwm', self.callback, 10)

    def callback(self, msg):
        if not self.ser: return
        pwm = msg.data
        # Protocol: Start(0xAA) + 4xPWM + Stop(0x55)
        try:
            packet = struct.pack('<BHHHHB', 0xAA, pwm[0], pwm[1], pwm[2], pwm[3], 0x55)
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().warn(f"Serial Write Error: {e}")
            # Optional: Try to reconnect here if you want to be fancy

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()