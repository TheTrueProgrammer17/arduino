import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class JoyToText(Node):
    def __init__(self):
        super().__init__('joy_to_text')

        self.ser = None
        # Try to connect to common ESP32 ports
        ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.get_logger().info(f"✅ Connected to ESP32 on {port}")
                time.sleep(2) # Wait for restart
                break
            except:
                continue

        if not self.ser:
            self.get_logger().error("❌ ESP32 NOT FOUND! Plug it in.")

        self.sub = self.create_subscription(Twist, '/rov/cmd_vel', self.callback, 10)
        self.last_cmd = 's'
        self.get_logger().info("⌨️  Ready! Moving Joystick sends: f, b, l, r, u, a, d")

    def callback(self, msg):
        if not self.ser: return

        # Threshold to trigger command
        LIMIT = 0.5 
        cmd = 's' # Default to STOP

        # --- LOGIC MATCHING YOUR GOLDEN CODE ---

        # 1. Forward / Backward (Left Stick Y)
        if msg.linear.x > LIMIT:      cmd = 'f' 
        elif msg.linear.x < -LIMIT:   cmd = 'b' 
        
        # 2. Left / Right Turn (Left Stick X)
        elif msg.angular.z > LIMIT:   cmd = 'l' 
        elif msg.angular.z < -LIMIT:  cmd = 'r' 
        
        # 3. UP (Right Stick Up)
        # Note: Your code ONLY has 'u'. It has no 'down'.
        elif msg.linear.z > LIMIT:    cmd = 'u' 
        
        # 4. SWAY (Right Stick X)
        # Your code maps 'd' -> Sway Right, 'a' -> Sway Left
        elif msg.linear.y < -LIMIT:   cmd = 'd' # Sway Right
        elif msg.linear.y > LIMIT:    cmd = 'a' # Sway Left

        # Only send if the command changed (don't spam 'f' 100 times a second)
        if cmd != self.last_cmd:
            try:
                # Send command with newline (\n) because your code uses readStringUntil('\n')
                payload = f"{cmd}\n"
                self.ser.write(payload.encode('utf-8'))
                self.get_logger().info(f"Sent: {cmd}")
                self.last_cmd = cmd
            except Exception as e:
                self.get_logger().error(f"Serial Write Failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyToText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()