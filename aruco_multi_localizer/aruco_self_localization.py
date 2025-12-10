import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import tf_transformations
import yaml
import math

class ArucoSelfLocalization(Node):
    def __init__(self):
        super().__init__("self_localization")

        # Parameters
        self.declare_parameter("camera_calibration_file", "")
        self.declare_parameter("square_size", 0.02)
        self.declare_parameter("map_file", "aruco_map.yaml")

        # Load calibration
        calib_file = self.get_parameter("camera_calibration_file").get_parameter_value().string_value
        a = np.load(calib_file)
        self.camera_matrix = a["camera_matrix"]
        self.dist_coeffs = a["dist_coeffs"]

        self.square_size = float(self.get_parameter("square_size").value)

        # Load marker map
        map_file = self.get_parameter("map_file").get_parameter_value().string_value
        with open(map_file, "r") as f:
            self.map_data = yaml.safe_load(f)["aruco_markers"]

        # Marker 3D geometry (4 corners)
        s = self.square_size
        self.object_points = np.array([
            [0,   0, 0],
            [8*s, 0, 0],
            [8*s, 8*s, 0],
            [0, 8*s, 0]
        ], dtype=np.float32)

        # ROS infra
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(Image, "video_frames", self.callback, 10)
        self.pub_aruco = self.create_publisher(Image, "aruco_detection", 10)
        self.pub_axes = self.create_publisher(Image, "coordinate_system", 10)

        self.pose_pub = self.create_publisher(PoseStamped, "camera_pose", 10)
        self.path_pub = self.create_publisher(Path, "camera_path", 10)

        self.path = Path()
        self.path.header.frame_id = "map"

        # Aruco Dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info("Task 3: Multi-marker camera localization initialized.")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        self.pub_aruco.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        camera_poses_world = []

        # For each detected marker:
        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)

            if marker_id not in self.map_data:
                continue

            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                corners[i][0],
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_SQPNP
            )

            if not success:
                continue

            # Convert PnP result (marker→camera) to (camera→marker)
            cam_pos_marker, rot_marker = self.invert_transform(rvec, tvec)

            # Convert camera pose to world frame
            cam_world = self.marker_to_world(marker_id, cam_pos_marker, rot_marker)
            camera_poses_world.append(cam_world)

            # Draw axes on the frame
            frame = self.draw_axes(frame, rvec, tvec)

        # If multiple markers detected → fuse by averaging
        if len(camera_poses_world) == 0:
            return

        fused_pose = np.mean(np.array(camera_poses_world), axis=0)

        # Publish TF, Pose, and Path
        self.publish_tf_and_pose(fused_pose)
        self.update_path(fused_pose)

        self.pub_axes.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    # ----- Transform utilities -----

    def invert_transform(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        R_inv = R.T
        t_inv = -R_inv @ tvec
        return t_inv, R_inv

    def marker_to_world(self, marker_id, t_cam_marker, R_cam_marker):
        m = self.map_data[str(marker_id)]
        x,y,z = m["x"], m["y"], m["z"]
        roll,pitch,yaw = m["roll"], m["pitch"], m["yaw"]

        R_world = self.euler_to_rot(roll, pitch, yaw)
        T_world = np.array([x,y,z]).reshape(3,1)

        R_cam_world = R_world @ R_cam_marker
        T_cam_world = R_world @ t_cam_marker + T_world

        quat = tf_transformations.quaternion_from_matrix(self.rot_to_mat4(R_cam_world))

        return [T_cam_world[0,0], T_cam_world[1,0], T_cam_world[2,0],
                quat[0], quat[1], quat[2], quat[3]]

    def publish_tf_and_pose(self, pose):
        x,y,z, qx,qy,qz,qw = pose

        # TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "camera"
        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = float(z)
        tf_msg.transform.rotation.x = float(qx)
        tf_msg.transform.rotation.y = float(qy)
        tf_msg.transform.rotation.z = float(qz)
        tf_msg.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(tf_msg)

        # PoseStamped
        ps = PoseStamped()
        ps.header = tf_msg.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

    def update_path(self, pose):
        x,y,z, qx,qy,qz,qw = pose
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        self.path.poses.append(ps)
        self.path_pub.publish(self.path)

    # ---- Rotation utilities -----

    def euler_to_rot(self, r,p,y):
        cx, sx = math.cos(r), math.sin(r)
        cy, sy = math.cos(p), math.sin(p)
        cz, sz = math.cos(y), math.sin(y)
        return np.array([
            [cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx],
            [sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx],
            [  -sy,           cy*sx,           cy*cx]
        ])

    def rot_to_mat4(self, R):
        M = np.eye(4)
        M[:3,:3] = R
        return M

    # ----- Drawing -----

    def draw_axes(self, frame, rvec, tvec):
        axis = np.float32([[0,0,0],[0.1,0,0],[0,0.1,0],[0,0,-0.1]])
        imgpts,_ = cv2.projectPoints(axis, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        origin = tuple(imgpts[0].ravel().astype(int))
        frame = cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0,0,255), 3)
        frame = cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0,255,0), 3)
        frame = cv2.line(frame, origin, tuple(imgpts[3].ravel()), (255,0,0), 3)
        return frame


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSelfLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

