import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Bool, Int32 # Added Int32
import serial

class ROVThrusterController(Node):
    def __init__(self):
        super().__init__("rov_thruster_controller")

        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
            self.get_logger().info("Connected to Arduino Uno")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

        self.PWM_NEUTRAL = 1500
        self.PWM_MIN     = 1100
        self.PWM_MAX     = 1900
        
        self.surge = 0.0
        self.yaw = 0.0
        self.heave = 0.0
        self.is_armed = False         
        self.emergency_stop = False   
        self.depth_lock_active = False
        self.locked_heave_val = 0.0
        self.arm_angle = 90 # Default start angle for the ring-picking arm

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, "/rov/cmd_vel", self.cmd_callback, 10)
        self.arm_sub = self.create_subscription(Bool, "/rov/arm_status", self.arm_callback, 10)
        self.lock_sub = self.create_subscription(Bool, "/rov/depth_lock", self.lock_callback, 10)
        self.stop_sub = self.create_subscription(Bool, "/rov/emergency_stop", self.stop_callback, 10)
        self.arm_angle_sub = self.create_subscription(Int32, "/rov/arm_angle", self.arm_angle_callback, 10) # New Sub
        
        self.pwm_pub = self.create_publisher(Int16MultiArray, "/rov/thruster_pwm", 10)
        self.timer = self.create_timer(0.05, self.update_mixer)
        self.get_logger().info("ROV Controller Active. Waiting for Arm...")

    # ... [Keep cmd_callback, arm_callback, lock_callback, stop_callback exactly as they were] ...

    def cmd_callback(self, msg):
        self.surge = msg.linear.x
        self.yaw = msg.angular.z
        if not self.depth_lock_active:
            self.heave = msg.linear.z

    def arm_callback(self, msg):
        self.is_armed = msg.data
        status = "ARMED" if self.is_armed else "DISARMED"
        self.get_logger().info(f"Status changed: {status}")

    def lock_callback(self, msg):
        if self.emergency_stop or not self.is_armed: return 
        if msg.data and not self.depth_lock_active:
            self.depth_lock_active = True
            self.locked_heave_val = self.heave
        elif not msg.data:
            self.depth_lock_active = False

    def stop_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().fatal("🚨 EMERGENCY STOP ACTIVATED 🚨")
            self.is_armed = False
            self.depth_lock_active = False

    def arm_angle_callback(self, msg):
        # Update current angle safely between 0 and 180
        self.arm_angle = max(0, min(180, msg.data))

    def map_pwm(self, val):
        val = max(-1.0, min(1.0, val))
        if val > 0:
            return int(self.PWM_NEUTRAL + (val * (self.PWM_MAX - self.PWM_NEUTRAL)))
        else:
            return int(self.PWM_NEUTRAL + (val * (self.PWM_NEUTRAL - self.PWM_MIN)))

    def update_mixer(self):
        if self.emergency_stop or not self.is_armed:
            stop_pwm = [1500, 1500, 1000, 1000]
            # Send 6 values (Thrusters + Arm Servos)
            full_payload = stop_pwm + [self.arm_angle, self.arm_angle] 
            self.send_to_arduino(full_payload)
            self.publish_pwm_msg(stop_pwm)
            return

        curr_heave = self.locked_heave_val if self.depth_lock_active else self.heave
        
        h_left  = self.surge + (self.yaw * 0.5)
        h_right = self.surge - (self.yaw * 0.5)
        v_thrust = max(0.0, curr_heave) 

        final_pwm = [
            self.map_pwm(h_left), 
            self.map_pwm(h_right), 
            int(1000 + v_thrust * 1000), 
            int(1000 + v_thrust * 1000)
        ]
        
        # Append the arm angles to the payload sent to Arduino
        full_payload = final_pwm + [self.arm_angle, self.arm_angle]
        self.send_to_arduino(full_payload)
        self.publish_pwm_msg(final_pwm)

        sw_state = "LCK" if self.depth_lock_active else ("ARM" if self.is_armed else "DIS")
        # Added Arm Angle to the logger output
        self.get_logger().info(
            f"SW:{sw_state} S:{self.surge:.2f} H:{curr_heave:.2f} | "
            f"L:{final_pwm[0]} R:{final_pwm[1]} V1:{final_pwm[2]} V2:{final_pwm[3]} ARM:{self.arm_angle}°"
        )

    def send_to_arduino(self, values):
        if self.arduino:
            data = ",".join(map(str, values)) + "\n"
            self.arduino.write(data.encode())

    def publish_pwm_msg(self, pwm_values):
        msg = Int16MultiArray()
        msg.data = pwm_values
        self.pwm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ROVThrusterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()