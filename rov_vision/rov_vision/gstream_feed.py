import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GStreamerCamera(Node):
    def __init__(self):
        super().__init__('gstreamer_camera')
        self.publisher_ = self.create_publisher(Image, '/rov/camera', 10)
        self.bridge = CvBridge()

        # Replace '0' with your actual video number if it wasn't /dev/video0
        # cv2.CAP_V4L2 forces OpenCV to use the standard Linux camera driver
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        # Optional: Ask OpenCV to set the resolution, rather than forcing it
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera via V4L2!")

        self.timer = self.create_timer(0.03, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning("Failed to grab frame from camera", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = GStreamerCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()