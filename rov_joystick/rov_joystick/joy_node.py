import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32 # Added Int32
import pygame

class ROVJoyNode(Node):
    def __init__(self):
        super().__init__("rov_joy_node")
        
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
        else:
            self.controller = None

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/rov/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Bool, '/rov/arm_status', 10)
        self.lock_pub = self.create_publisher(Bool, '/rov/depth_lock', 10)
        self.stop_pub = self.create_publisher(Bool, '/rov/emergency_stop', 10)
        self.arm_angle_pub = self.create_publisher(Int32, '/rov/arm_angle', 10) # New Pub

        self.timer = self.create_timer(0.05, self.publish_cmd) # Sped up slightly for smoother arm control
        
        # --- BUTTON MAPPING ---
        self.AXIS_SURGE = 1  
        self.AXIS_YAW   = 0  
        self.AXIS_HEAVE = 4  

        self.BTN_ARM    = 0  # 'A'
        self.BTN_STOP   = 1  # 'B'
        self.BTN_LOCK   = 3  # 'Y'
        
        # New Arm Controls (Standard Xbox/Logitech mapping)
        self.BTN_ARM_UP   = 4 # L1 (Left Bumper)
        self.BTN_ARM_DOWN = 5 # R1 (Right Bumper)

        # State Tracking
        self.armed = False
        self.depth_lock = False
        self.emergency_state = False
        self.current_arm_angle = 90 # Start at neutral 90 degrees
        
        self.last_states = { 'arm': 0, 'lock': 0, 'stop': 0 }

    def publish_cmd(self):
        if not self.controller: return
        pygame.event.pump()
        
        # --- 1. EMERGENCY STOP (B Button) ---
        curr_stop = self.controller.get_button(self.BTN_STOP)
        if curr_stop == 1 and self.last_states['stop'] == 0:
            self.emergency_state = not self.emergency_state
            self.stop_pub.publish(Bool(data=self.emergency_state))
            if self.emergency_state: self.armed = False
        self.last_states['stop'] = curr_stop

        # --- 2. ARMING (A Button) ---
        curr_arm = self.controller.get_button(self.BTN_ARM)
        if curr_arm == 1 and self.last_states['arm'] == 0:
            if not self.emergency_state:
                self.armed = not self.armed
                self.arm_pub.publish(Bool(data=self.armed))
        self.last_states['arm'] = curr_arm

        # --- 3. DEPTH LOCK (Y Button) ---
        curr_lock = self.controller.get_button(self.BTN_LOCK)
        if curr_lock == 1 and self.last_states['lock'] == 0:
            if self.armed and not self.emergency_state:
                self.depth_lock = not self.depth_lock
                self.lock_pub.publish(Bool(data=self.depth_lock))
        self.last_states['lock'] = curr_lock

        # --- 4. ROBOTIC ARM CONTROL (L1 / R1) ---
        if self.armed and not self.emergency_state:
            # Increment or decrement angle
            if self.controller.get_button(self.BTN_ARM_UP):
                self.current_arm_angle += 2
            elif self.controller.get_button(self.BTN_ARM_DOWN):
                self.current_arm_angle -= 2
                
            # Keep within servo limits
            self.current_arm_angle = max(0, min(180, self.current_arm_angle))
            self.arm_angle_pub.publish(Int32(data=self.current_arm_angle))

        # --- 5. MOVEMENT ---
        msg = Twist()
        if self.armed and not self.emergency_state:
            msg.linear.x = self.deadzone(-self.controller.get_axis(self.AXIS_SURGE))
            msg.angular.z = self.deadzone(self.controller.get_axis(self.AXIS_YAW))
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