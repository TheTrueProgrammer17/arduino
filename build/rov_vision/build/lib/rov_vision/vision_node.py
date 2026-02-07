import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Initialize GStreamer first
Gst.init(None)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from queue import Queue, Empty

class ROVVisionNode(Node):
    def __init__(self):
        super().__init__('rov_vision__copy_node')
        self.publisher_ = self.create_publisher(Image, 'rov/camera_stream', 10)
        self.bridge = CvBridge()
        self.frame_queue = Queue(maxsize=2)

        # GStreamer Pipeline
        pipeline_str = (
            "v4l2src device=/dev/video2 ! "
            "image/jpeg, width=640, height=480, framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "videobalance contrast=1.3 brightness=-0.1 saturation=1.2 ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true "
            "caps=video/x-raw,format=BGR"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.connect("new-sample", self.on_new_sample)

        self.pipeline.set_state(Gst.State.PLAYING)
        
        # ROS Timer for processing (30 FPS)
        self.create_timer(0.03, self.publish_and_show)
        
        self.get_logger().info("ROV Vision Online - Press 'q' in the window to close GUI")

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if not sample: return Gst.FlowReturn.ERROR
        
        buf = sample.get_buffer()
        data = buf.extract_dup(0, buf.get_size())
        s = sample.get_caps().get_structure(0)
        w, h = s.get_value("width"), s.get_value("height")
        
        frame = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
        
        if not self.frame_queue.full():
            self.frame_queue.put(frame)
        return Gst.FlowReturn.OK

    def apply_filters(self, frame):
        # Underwater enhancement logic
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        frame = cv2.cvtColor(cv2.merge((cl, a, b)), cv2.COLOR_LAB2BGR)
        frame[:, :, 2] = np.clip(frame[:, :, 2] * 1.5, 0, 255).astype(np.uint8)
        return frame

    def publish_and_show(self):	
        try:
            frame = self.frame_queue.get_nowait()
            processed = self.apply_filters(frame)
            
            # 1. Show the window
            cv2.imshow("ROV Vision Feed", processed)
            
            # 2. WaitKey is REQUIRED for the window to render
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Closing window...")
                cv2.destroyAllWindows()

            # 3. Publish to ROS
            msg = self.bridge.cv2_to_imgmsg(processed, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
            
        except Empty:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ROVVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()