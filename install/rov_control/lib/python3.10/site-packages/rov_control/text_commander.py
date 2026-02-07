import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class TextCommander(Node):
    def __init__(self):
        super().__init__('text_commander')

        self.ser = None
        # Auto-connect to ESP32
        for port in ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.get_logger().info(f"✅ Connected to ESP32 on {port}")
                break
            except:
                continue

        if not self.ser:
            self.get_logger().error("❌ ESP32 NOT FOUND! Plug it in.")

        self.sub = self.create_subscription(Twist, '/rov/cmd_vel', self.callback, 10)
        self.last_cmd = 's'

    def callback(self, msg):
        if not self.ser or not self.ser.is_open: return

        LIMIT = 0.5 
        cmd = 's' # Default Stop

        # --- UPDATED LOGIC FOR YOUR ROV ---
        
        # 1. Surge (Forward/Back)
        if msg.linear.x > LIMIT:      cmd = 'f' 
        elif msg.linear.x < -LIMIT:   cmd = 'b' 
        
        # 2. Yaw (Turning) - SWAPPED LEFT/RIGHT
        # Left Stick Left -> Send 'r' (Right command) to fix reversed motors
        elif msg.angular.z > LIMIT:   cmd = 'r' 
        # Left Stick Right -> Send 'l' (Left command) to fix reversed motors
        elif msg.angular.z < -LIMIT:  cmd = 'l' 
        
        # 3. Heave (Depth) - Fixed Up/Down
        elif msg.linear.z < -LIMIT:   cmd = 'd' # Stick Down -> Dive
        elif msg.linear.z > LIMIT:    cmd = 'u' # Stick Up -> Surface

        if cmd != self.last_cmd:
            self.ser.write(f"{cmd}\n".encode('utf-8'))
            self.get_logger().info(f"Sent: {cmd}")
            self.last_cmd = cmd

def main(args=None):
    rclpy.init(args=args)
    node = TextCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()