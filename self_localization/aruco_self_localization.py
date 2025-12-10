import rclpy
from cv2 import SOLVEPNP_ITERATIVE, SOLVEPNP_IPPE_SQUARE, SOLVEPNP_SQPNP
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
        self.declare_parameter("camera_calibration_file", "/home/jmichael2/cas_data/camera_calibration_params.npz")
        calib_file = self.get_parameter("camera_calibration_file").get_parameter_value().string_value
        a = np.load(calib_file)
        self.camera_matrix = a['camera_matrix']
        self.dist_coeffs = a['dist_coeffs']
        self.declare_parameter("square_size", 0.0255)
        self.square_size = self.get_parameter("square_size").get_parameter_value().double_value
        self.object_points = np.array([
            [0, 0, 0],  # Top-left corner (origin)
            [8 * self.square_size, 0, 0],  # Top-right corner
            [8 * self.square_size, 8 * self.square_size, 0],  # Bottom-right corner
            [0,8 * self.square_size, 0]  # Bottom-left corner
        ], dtype=np.float32)

        self.create_subscription(msg_type=Image, topic="video_frames", callback=self.self_localize, qos_profile=10)
        self.publisher_aruco = self.create_publisher(Image, "aruco_detection", 10)
        self.publisher_coordinate_system = self.create_publisher(Image, "coordinate_system", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.rvec = None
        self.tvec = None
        self.get_logger().info("OpenCV Version: " + cv2.__version__)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def self_localize(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids = self.detect_aruco_pattern(frame)
        if ids is not None:
            image_points = corners[0][0].reshape(-1, 2)
            success, rvec, tvec= self.solve_pnp(image_points)
            if success:
                self.publish_frame_on_image(frame, image_points, rvec, tvec)
                self.publish_pose(rvec, tvec)

    def detect_aruco_pattern(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary=self.aruco_dict, parameters=self.aruco_params)
        cv2.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids)
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_aruco.publish(msg)
        return corners, ids

    def solve_pnp(self, image_points):
        success, rvec, tvec = cv2.solvePnP(objectPoints=self.object_points, imagePoints=image_points,
                                                     cameraMatrix=self.camera_matrix, distCoeffs=self.dist_coeffs,
                                                     flags=SOLVEPNP_SQPNP)
        return success, rvec, tvec

    def publish_frame_on_image(self, frame, image_points, rvec, tvec):
        # Draw the 3D axis on the frame for visualization
        axis = np.float32([[0, 0, 0],
                           [8 * self.square_size, 0, 0],
                           [0, 8 * self.square_size, 0],
                           [0.0, 0.0, 8 * self.square_size]])
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        frame = self.draw_axis(frame, image_points[0], imgpts)
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_coordinate_system.publish(msg)

    def publish_pose(self, rvec, tvec):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "aruco_pattern"
        msg.child_frame_id = "camera"
        # Get the transformations taking care about the fact that the solution of the pnp problem delivers the
        # inverse of the desired transformation
        t, rot_mat = self.get_transformation(tvec, rvec)
        # Fill the Translation part of the message
        msg.transform.translation.x = t[0, 0]
        msg.transform.translation.y = t[1, 0]
        msg.transform.translation.z = t[2, 0]
        # Convert to Quaternion format
        quat = tf_transformations.quaternion_from_matrix(rot_mat)
        # Fill the Rotation part of the message
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(msg)

    def get_transformation(self, tvec, rvec):
        rot_mat = np.zeros([4, 4], dtype=np.float32)
        rot_mat[3, 3] = 1.0
        rot_mat[:3, :3], _ = cv2.Rodrigues(rvec)
        rot_mat[:3, :3] = rot_mat[:3, :3].transpose()
        t = -rot_mat[:3, :3] @ tvec
        return t, rot_mat

    def draw_axis(self, img, corner, imgpts):
        """Draw the 3D axis on the image."""
        corner = tuple(corner.ravel().astype(np.int32))
        img = cv2.line(img, corner, tuple(imgpts[1].ravel().astype(np.int32)), (0, 0, 255), 5)  # X-axis in red
        img = cv2.line(img, corner, tuple(imgpts[2].ravel().astype(np.int32)), (0, 255, 0), 5)  # Y-axis in green
        img = cv2.line(img, corner, tuple(imgpts[3].ravel().astype(np.int32)), (255, 0, 0), 5)  # Z-axis in blue
        return img


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
