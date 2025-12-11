import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
import tf_transformations


class MultiArucoLocalization(Node):
    def __init__(self):
        super().__init__("multi_aruco_localization")
        self.declare_parameter("camera_calibration_file", "/home/jmichael2/cas_data/camera_calibration_params.npz")
        calib_file = self.get_parameter("camera_calibration_file").get_parameter_value().string_value
        calib = np.load(calib_file)
        self.camera_matrix = calib["camera_matrix"]
        self.dist_coeffs = calib["dist_coeffs"]

        self.declare_parameter("aruco_map_file", "config/aruco_map.yaml")
        self.declare_parameter("marker_size", 0.04)
        self.declare_parameter("map_angles_degrees", False)
        self.declare_parameter("debug_logging", False)
        map_file = self.get_parameter("aruco_map_file").get_parameter_value().string_value
        marker_size_default = self.get_parameter("marker_size").get_parameter_value().double_value
        angles_in_degrees = self.get_parameter("map_angles_degrees").get_parameter_value().bool_value
        self.debug_logging = self.get_parameter("debug_logging").get_parameter_value().bool_value
        self.world_frame, self.marker_size, self.marker_transforms = self.load_marker_map(
            map_file, marker_size_default, angles_in_degrees
        )

        # Size of the detected ArUco square (meters).
        self.object_points = np.array(
            [
                [0.0, 0.0, 0.0],
                [self.marker_size, 0.0, 0.0],
                [self.marker_size, self.marker_size, 0.0],
                [0.0, self.marker_size, 0.0],
            ],
            dtype=np.float32,
        )

        self.create_subscription(Image, "video_frames", self.self_localize, 10)
        self.publisher_aruco = self.create_publisher(Image, "aruco_detection", 10)
        self.publisher_coordinate_system = self.create_publisher(Image, "coordinate_system", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "camera_pose", 10)
        self.path_pub = self.create_publisher(Path, "camera_path", 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame

        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def load_marker_map(self, map_file, marker_size_default, angles_in_degrees):
        with open(map_file, "r") as f:
            map_data = yaml.safe_load(f)

        world_frame = map_data.get("world_frame", "map")
        marker_size = float(map_data.get("marker_size", marker_size_default))

        marker_transforms = {}
        if "markers" in map_data:
            # Legacy format: list of {id, position, rpy (deg)}
            for marker in map_data.get("markers", []):
                marker_id = int(marker["id"])
                pos = np.array(marker["position"], dtype=np.float32)
                rpy = np.deg2rad(marker.get("rpy", [0.0, 0.0, 0.0]))
                rot = tf_transformations.euler_matrix(rpy[0], rpy[1], rpy[2])
                rot[0:3, 3] = pos
                marker_transforms[marker_id] = rot
        elif "aruco_markers" in map_data:
            # Format from cooperative-system repo: dict {id: {x,y,z, roll,pitch,yaw}}
            for marker_id_key, m in map_data["aruco_markers"].items():
                marker_id = int(marker_id_key)
                pos = np.array([m.get("x", 0.0), m.get("y", 0.0), m.get("z", 0.0)], dtype=np.float32)
                roll = m.get("roll", 0.0)
                pitch = m.get("pitch", 0.0)
                yaw = m.get("yaw", 0.0)
                if angles_in_degrees:
                    roll, pitch, yaw = np.deg2rad([roll, pitch, yaw])
                rot = tf_transformations.euler_matrix(roll, pitch, yaw)
                rot[0:3, 3] = pos
                marker_transforms[marker_id] = rot
        else:
            raise ValueError("Marker map format not recognized. Provide 'markers' or 'aruco_markers'.")

        return world_frame, marker_size, marker_transforms

    def detect_aruco_pattern(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary=self.aruco_dict, parameters=self.aruco_params)
        cv2.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids)
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_aruco.publish(msg)
        return corners, ids

    def solve_pnp(self, image_points):
        return cv2.solvePnP(
            objectPoints=self.object_points,
            imagePoints=image_points,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
            flags=cv2.SOLVEPNP_SQPNP,
        )

    def self_localize(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids = self.detect_aruco_pattern(frame)
        if ids is None:
            return
        if self.debug_logging:
            self.get_logger().info(f"Detected IDs: {ids.flatten().tolist()}")

        valid_transforms = []
        for idx, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            if marker_id not in self.marker_transforms:
                if self.debug_logging:
                    self.get_logger().warn(f"Marker {marker_id} not in map, skipping.")
                continue

            image_points = corners[idx][0].reshape(-1, 2)
            success, rvec, tvec = self.solve_pnp(image_points)
            if not success:
                if self.debug_logging:
                    self.get_logger().warn(f"solvePnP failed for marker {marker_id}")
                continue

            t_cam_marker = self.to_homogeneous(rvec, tvec)
            t_marker_cam = np.linalg.inv(t_cam_marker)
            t_world_marker = self.marker_transforms[marker_id]
            t_world_cam = t_world_marker @ t_marker_cam
            valid_transforms.append((marker_id, t_world_cam, image_points, rvec, tvec))

        if not valid_transforms:
            return

        marker_id, t_world_cam, image_points, rvec, tvec = valid_transforms[0]
        if self.debug_logging:
            pos = t_world_cam[0:3, 3]
            self.get_logger().info(f"Using marker {marker_id}, world->cam pos: {pos}")
        self.publish_frame_on_image(frame, image_points, rvec, tvec)
        self.publish_world_pose(t_world_cam, msg.header.stamp)

    def publish_frame_on_image(self, frame, image_points, rvec, tvec):
        axis_len = self.marker_size
        axis = np.float32(
            [
                [0, 0, 0],
                [axis_len, 0, 0],
                [0, axis_len, 0],
                [0.0, 0.0, axis_len],
            ]
        )
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        frame = self.draw_axis(frame, image_points[0], imgpts)
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_coordinate_system.publish(msg)

    def publish_world_pose(self, t_world_cam, stamp):
        quat = tf_transformations.quaternion_from_matrix(t_world_cam)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = float(t_world_cam[0, 3])
        pose.pose.position.y = float(t_world_cam[1, 3])
        pose.pose.position.z = float(t_world_cam[2, 3])
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.pose_pub.publish(pose)

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = "camera"
        msg.transform.translation.x = pose.pose.position.x
        msg.transform.translation.y = pose.pose.position.y
        msg.transform.translation.z = pose.pose.position.z
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(msg)

    def draw_axis(self, img, corner, imgpts):
        corner = tuple(corner.ravel().astype(np.int32))
        img = cv2.line(img, corner, tuple(imgpts[1].ravel().astype(np.int32)), (0, 0, 255), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel().astype(np.int32)), (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[3].ravel().astype(np.int32)), (255, 0, 0), 5)
        return img

    def to_homogeneous(self, rvec, tvec):
        rot_mat, _ = cv2.Rodrigues(rvec)
        t_cam_marker = np.eye(4, dtype=np.float32)
        t_cam_marker[:3, :3] = rot_mat
        t_cam_marker[:3, 3] = tvec.reshape(3)
        return t_cam_marker


def main(args=None):
    rclpy.init(args=args)

    node = MultiArucoLocalization()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
