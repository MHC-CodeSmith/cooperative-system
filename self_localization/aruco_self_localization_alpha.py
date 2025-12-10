import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations


class ArucoSelfLocalization(Node):
    def __init__(self):
        super().__init__("self_localization")
        self.create_subscription(msg_type=Image, topic="video_frames", callback=self.self_localize, qos_profile=10)
        self.publisher_aruco = self.create_publisher(Image, "aruco_detection", 10)
        self.publisher_coordinate_system = self.create_publisher(Image, "coordinate_system", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def self_localize(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        corners, ids = self.detect_arucoPattern(frame)
        if ids is not None:
            success, rvec, tvec = self.solvePnP(frame, corners, ids)
            if success:
                self.publish_pose(rvec, tvec)

    def publish_pose(self, rvec, tvec):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "camera"
        msg.transform.translation.x = tvec[0, 0]
        msg.transform.translation.y = tvec[1, 0]
        msg.transform.translation.z = tvec[2, 0]
        rot_mat = np.zeros([4,4], dtype=np.float32)
        rot_mat[3,3 ] = 1.0
        rot_mat[:3, :3], _= cv2.Rodrigues(rvec)
        rot_around_z = tf_transformations.quaternion_from_euler(0, np.deg2rad(180), 0)
        quat = tf_transformations.quaternion_from_matrix(rot_mat)
        quat = tf_transformations.quaternion_multiply(quat, rot_around_z)
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(msg)

    def draw_axis(self, img, corner, imgpts):
        """Draw the 3D axis on the image."""
        corner = tuple(corner.ravel().astype(np.int32))
        img = cv2.line(img, corner, tuple(imgpts[1].ravel().astype(np.int32)), (0, 0, 255), 5)  # X-axis in red
        img = cv2.line(img, corner, tuple(imgpts[2].ravel().astype(np.int32)), (0, 255, 0), 5)  # Y-axis in green
        img = cv2.line(img, corner, tuple(imgpts[3].ravel().astype(np.int32)), (255, 0, 0), 5)  # Z-axis in blue
        return img

    def solvePnP(self, frame, corners, ids):
        image_points = corners[0][0].reshape(-1, 2)
        success, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs)
        if success:
            # Draw the 3D axis on the frame for visualization
            axis = np.float32([[0, 0, 0],
                               [2 * self.square_size, 0, 0],
                               [0, 2 * self.square_size, 0],
                               [0.0, 0.0, -2 * self.square_size]])
            imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            frame = self.draw_axis(frame, image_points[0], imgpts)
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_coordinate_system.publish(msg)
        return success, rvec, tvec

    def detect_arucoPattern(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        cv2.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids)
        # msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        # self.publisher_aruco.publish(msg)
        return corners, ids

def main(args=None):
    rclpy.init(args=args)

    self_localization_node = ArucoSelfLocalization()

    try:
        rclpy.spin(self_localization_node)
    except KeyboardInterrupt:
        pass

    self_localization_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
