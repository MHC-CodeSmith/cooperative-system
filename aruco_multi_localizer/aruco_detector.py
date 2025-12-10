import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
import os
import tf_transformations as tf
from .camera_localizer import estimate_camera_pose

# Known marker poses
MARKER_GLOBAL_POSES = {
    0: ([0.0, 0.0, 0.0], [0, 0, 0]),
    1: ([0.5, 0.0, 0.0], [0, 0, 0]),
    2: ([0.0, 0.5, 0.0], [0, 0, np.pi/2]),
    3: ([1.0, 0.5, 0.0], [0, 0, 0]),
    4: ([1.5, 0.0, 0.0], [0, 0, 0]),
}

MARKER_SIZE = 0.06  # meters

class MultiArucoLocalizer(Node):
    def __init__(self):
        super().__init__('multi_aruco_localizer')
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, "/camera_path", 10)
        self.image_pub = self.create_publisher(Image, "/aruco_localization/image", 10)
        self.path = Path()

        # Load camera calibration
        pkg_path = get_package_share_directory('aruco_multi_localization')
        calib_file = os.path.join(pkg_path, "camera_calibration_params.npz")
        calib = np.load(calib_file)
        self.camera_matrix = calib["camera_matrix"]
        self.dist_coeffs = calib["dist_coeffs"]

        # ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        # Subscribe to camera
        self.sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        ids = ids.flatten()
        camera_poses = []

        # Estimate camera pose from each marker
        for i, marker_id in enumerate(ids):
            if marker_id not in MARKER_GLOBAL_POSES:
                continue

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], MARKER_SIZE, self.camera_matrix, self.dist_coeffs
            )

            R_wc, t_wc = estimate_camera_pose(
                marker_id, rvec[0], tvec[0], MARKER_GLOBAL_POSES
            )
            camera_poses.append((R_wc, t_wc))

            # Draw markers and axes
            cv2.aruco.drawDetectedMarkers(frame, [corners[i]])
            cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs,
                               rvec[0], tvec[0], 0.05)

        if not camera_poses:
            return

        # Use first camera pose
        R_wc, t_wc = camera_poses[0]

        # Publish TF
        from .tf_publisher import publish_tf
        publish_tf(self, R_wc, t_wc)

        # Publish path
        from .path_publisher import publish_path
        publish_path(self, R_wc, t_wc, self.path)

        # Publish annotated image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
