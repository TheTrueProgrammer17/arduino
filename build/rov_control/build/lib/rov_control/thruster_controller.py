import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Bool

class ROVThrusterController(Node):
    def __init__(self):
        super().__init__("rov_thruster_controller")

        # --- CONFIGURATION ---
        # T200 (Horizontal) -> Center is 1500 (Bidirectional)
        self.T200_NEUTRAL = 1500
        self.T200_RANGE   = 400  # 1100 to 1900

        # BLDC (Vertical) -> Start is 1000 (Unidirectional)
        self.BLDC_MIN     = 1000
        self.BLDC_MAX     = 2000
        
        self.surge = 0.0
        self.sway  = 0.0
        self.yaw   = 0.0
        self.heave = 0.0
        
        self.is_armed = False         
        self.emergency_stop = False   
        self.depth_lock_active = False
        self.locked_heave_val = 0.0

        # --- SUBSCRIBERS ---
        self.cmd_sub = self.create_subscription(Twist, "/rov/cmd_vel", self.cmd_callback, 10)
        self.arm_sub = self.create_subscription(Bool, "/rov/arm_status", self.arm_callback, 10)
        self.lock_sub = self.create_subscription(Bool, "/rov/depth_lock", self.lock_callback, 10)
        self.stop_sub = self.create_subscription(Bool, "/rov/emergency_stop", self.stop_callback, 10)
        
        self.pwm_pub = self.create_publisher(Int16MultiArray, "/rov/thruster_pwm", 10)
        
        self.timer = self.create_timer(0.05, self.update_mixer)
        self.get_logger().info("✅ HYBRID CONTROLLER READY: T200(Bi) + BLDC(Uni)")

    def cmd_callback(self, msg):
        self.surge = msg.linear.x
        self.sway  = msg.linear.y 
        self.yaw   = msg.angular.z
        if not self.depth_lock_active:
            self.heave = msg.linear.z

    def arm_callback(self, msg):
        self.is_armed = msg.data
        if self.is_armed: self.get_logger().info("🟢 SYSTEM ARMED")
        else: self.get_logger().info("🔒 SYSTEM DISARMED")

    def lock_callback(self, msg):
        if self.emergency_stop or not self.is_armed: return 
        if msg.data:
            self.depth_lock_active = True
            self.locked_heave_val = self.heave
            self.get_logger().info("⚓ DEPTH LOCK ON")
        else:
            self.depth_lock_active = False
            self.get_logger().info("⚓ DEPTH LOCK OFF")

    def stop_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.is_armed = False
            self.get_logger().fatal("🚨 EMERGENCY STOP! 🚨")

    # Map -1..1 to 1100..1900 (For T200s)
    def map_t200(self, val):
        val = max(-1.0, min(1.0, val))
        return int(self.T200_NEUTRAL + (val * self.T200_RANGE))

    # Map 0..1 to 1000..2000 (For BLDCs)
    def map_bldc(self, val):
        val = max(0.0, min(1.0, abs(val))) # Use abs() to ensure positive input
        return int(self.BLDC_MIN + (val * (self.BLDC_MAX - self.BLDC_MIN)))

    def update_mixer(self):
        if self.emergency_stop: return
            
        if not self.is_armed:
            # STOP STATE: T200=1500, BLDC=1000 (Important!)
            msg = Int16MultiArray()
            msg.data = [1500, 1500, 1000, 1000]
            self.pwm_pub.publish(msg)
            return

        curr_heave = self.locked_heave_val if self.depth_lock_active else self.heave
        
        # --- HORIZONTAL MIXER (T200) ---
        h_left  = self.surge - (self.yaw * 0.5) 
        h_right = self.surge + (self.yaw * 0.5)
        
        # Normalize to prevent clipping
        h_max = max(abs(h_left), abs(h_right))
        if h_max > 1.0: 
            h_left /= h_max
            h_right /= h_max

        pwm_left = self.map_t200(h_left)
        pwm_right = self.map_t200(h_right)

        # --- VERTICAL MIXER (Split BLDC) ---
        # Strategy: One motor for UP, One motor for DOWN
        
        pwm_v_left = 1000   # Default Stop
        pwm_v_right = 1000  # Default Stop

        if curr_heave > 0.1:  # SURFACE (Up)
            # Spin LEFT Vertical only (Push Down)
            pwm_v_left = self.map_bldc(curr_heave)
            pwm_v_right = 1000
            
        elif curr_heave < -0.1: # DIVE (Down)
            # Spin RIGHT Vertical only (Push Up)
            pwm_v_left = 1000
            pwm_v_right = self.map_bldc(curr_heave)

        # Publish
        msg = Int16MultiArray()
        msg.data = [pwm_left, pwm_right, pwm_v_left, pwm_v_right]
        self.pwm_pub.publish(msg)
        
        self.get_logger().info(
            f"S:{self.surge:.1f} H:{curr_heave:.1f} | T200:[{pwm_left},{pwm_right}] BLDC:[{pwm_v_left},{pwm_v_right}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ROVThrusterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()