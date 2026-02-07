import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import pygame

class ROVJoyNode(Node):
    def __init__(self):
        super().__init__("rov_joy_node")
        
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            self.get_logger().info(f"🎮 Joystick Connected: {self.controller.get_name()}")
        else:
            self.controller = None
            self.get_logger().warn("⚠️ NO JOYSTICK FOUND!")

        self.cmd_pub = self.create_publisher(Twist, '/rov/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Bool, '/rov/arm_status', 10)
        self.lock_pub = self.create_publisher(Bool, '/rov/depth_lock', 10)
        self.stop_pub = self.create_publisher(Bool, '/rov/emergency_stop', 10)

        self.timer = self.create_timer(0.1, self.publish_cmd)
        
        # --- CONFIGURATION ---
        self.AXIS_SURGE = 1  
        self.AXIS_SWAY  = 3  # Often Right Stick X
        self.AXIS_YAW   = 0  
        self.AXIS_HEAVE = 4  
        
        self.BTN_ARM    = 0  
        self.BTN_STOP   = 1  
        self.BTN_LOCK   = 3  

        self.armed = False
        self.depth_lock = False
        self.emergency_state = False
        self.last_states = { 'arm': 0, 'lock': 0, 'stop': 0 }

    def publish_cmd(self):
        if not self.controller: return
        pygame.event.pump()
        
        # EMERGENCY STOP
        curr_stop = self.controller.get_button(self.BTN_STOP)
        if curr_stop == 1 and self.last_states['stop'] == 0:
            self.emergency_state = not self.emergency_state
            self.stop_pub.publish(Bool(data=self.emergency_state))
            if self.emergency_state: 
                self.armed = False
                self.get_logger().error("🚨 EMERGENCY STOP TRIGGERED 🚨")
            else:
                self.get_logger().info("🚨 EMERGENCY STOP RELEASED")
        self.last_states['stop'] = curr_stop

        # ARMING
        curr_arm = self.controller.get_button(self.BTN_ARM)
        if curr_arm == 1 and self.last_states['arm'] == 0:
            if not self.emergency_state:
                self.armed = not self.armed
                self.arm_pub.publish(Bool(data=self.armed))
        self.last_states['arm'] = curr_arm

        # DEPTH LOCK
        curr_lock = self.controller.get_button(self.BTN_LOCK)
        if curr_lock == 1 and self.last_states['lock'] == 0:
            if self.armed and not self.emergency_state:
                self.depth_lock = not self.depth_lock
                self.lock_pub.publish(Bool(data=self.depth_lock))
        self.last_states['lock'] = curr_lock

        # MOVEMENT
        msg = Twist()
        if self.armed and not self.emergency_state:
            msg.linear.x = self.deadzone(-self.controller.get_axis(self.AXIS_SURGE))
            msg.linear.y = self.deadzone(self.controller.get_axis(self.AXIS_SWAY))
            msg.angular.z = self.deadzone(-self.controller.get_axis(self.AXIS_YAW))
            msg.linear.z = self.deadzone(-self.controller.get_axis(self.AXIS_HEAVE))
        
        self.cmd_pub.publish(msg)

    def deadzone(self, val):
        return 0.0 if abs(val) < 0.1 else val

def main(args=None):
    rclpy.init(args=args)
    node = ROVJoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()