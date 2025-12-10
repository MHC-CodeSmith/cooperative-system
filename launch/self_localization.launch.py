from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '2', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '3.1415', '--roll', '0', '--frame-id',
                       'map', '--child-frame-id', 'aruco_pattern']
        ),
        Node(
            package='self_localization',
            executable='image_publisher',
            parameters=[{"use_video": True},
                        {"cam_id": 0},
                        {"video_path": "/home/jmichael2/cas_data/aruco_example.mp4"}]
        ),
        Node(
            package='self_localization',
            executable='aruco_self_localization',
            parameters=[{"camera_calibration_file": "/home/jmichael2/cas_data/camera_calibration_params.npz"},
                        {"square_size": 0.014}]
        )

    ])