import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("self_localization")
    default_map = os.path.join(pkg_share, "config", "aruco_map.yaml")

    video_path = LaunchConfiguration("video_path")
    calibration_file = LaunchConfiguration("camera_calibration_file")
    map_file = LaunchConfiguration("aruco_map_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_path",
                default_value=os.path.expanduser("~/cas_data/multi_aruco.mp4"),
                description="Path to the recorded video with multiple ArUco markers.",
            ),
            DeclareLaunchArgument(
                "camera_calibration_file",
                default_value=os.path.expanduser("~/cas_data/camera_calibration_params.npz"),
                description="NumPy npz file with camera_matrix and dist_coeffs.",
            ),
            DeclareLaunchArgument(
                "aruco_map_file",
                default_value=default_map,
                description="YAML map describing marker poses in the world frame.",
            ),
            Node(
                package="self_localization",
                executable="image_publisher",
                parameters=[
                    {"use_video": True},
                    {"cam_id": 0},
                    {"video_path": video_path},
                ],
                output="screen",
            ),
            Node(
                package="self_localization",
                executable="multi_aruco_localization",
                parameters=[
                    {"camera_calibration_file": calibration_file},
                    {"aruco_map_file": map_file},
                ],
                output="screen",
            ),
        ]
    )
