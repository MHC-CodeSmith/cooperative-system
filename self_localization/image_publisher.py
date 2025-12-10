import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__("video_publisher")
        self.declare_parameter("use_video", True)
        use_video = self.get_parameter("use_video").get_parameter_value().bool_value
        self.declare_parameter("cam_id", 0)
        cam_id = self.get_parameter("cam_id").get_parameter_value().integer_value
        self.declare_parameter("video_path", "/home/jmichael2/cas_data/aruco_example.mp4")  # Declare the parameter with a default video file path
        self.video_path = self.get_parameter("video_path").get_parameter_value().string_value
        print(self.video_path)
        if use_video:
            self.cap = cv2.VideoCapture(self.video_path)
        else:
            self.cap = cv2.VideoCapture(cam_id)

        # Open the video file
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video file: {self.video_path}")
            rclpy.shutdown()

        # Retrieve the video FPS
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        fps = 10
        if fps <= 0:
            self.get_logger().warn("Could not retrieve FPS from the video. Using default 30 FPS.")
            fps = 30  # Default FPS in case of an error

        # Create the Publisher
        self.publisher_ = self.create_publisher(Image, "video_frames", 10)
        self.bridge = CvBridge()
        # Set the timer interval to match the video FPS
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().debug(f"Publishing video at {fps:.2f} FPS.")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            # Reset video to the beginning to loop it
            self.get_logger().info("End of video reached. Restarting video loop.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to read frame after restarting video.")
                rclpy.shutdown()
                return

        # Convert OpenCV image (BGR) to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(msg)
        self.get_logger().debug("Published video frame.")

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    video_publisher_node = ImagePublisherNode()

    try:
        rclpy.spin(video_publisher_node)
    except KeyboardInterrupt:
        pass

    video_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
