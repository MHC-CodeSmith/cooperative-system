import rclpy
from aruco_multi_localizer.aruco_detector import MultiArucoLocalizer

def main():
    rclpy.init()
    node = MultiArucoLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
