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

        gst_pipeline = (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw,width=640,height=480,framerate=30/1 ! "
            "videoconvert ! appsink drop=true sync=false"
        )

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open GStreamer pipeline")

        self.timer = self.create_timer(0.03, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GStreamerCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
