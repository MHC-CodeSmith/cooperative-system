#!/usr/bin/env python3
"""
ArUco Marker Generator for ROS2 project

Generates ArUco markers (IDs 0-4) using OpenCV and saves them
in the 'markers/' folder inside this package.
"""

import cv2
import numpy as np
import os

# Configuration
DESIRED_DICT = "DICT_6X6_250"
NUM_MARKERS = 5
MARKER_SIZE_PIXELS = 300

# Folder to save markers
SAVE_FOLDER = os.path.join(os.path.dirname(os.path.dirname(__file__)), "markers")
os.makedirs(SAVE_FOLDER, exist_ok=True)

# OpenCV ArUco dictionaries
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

def main():
    if DESIRED_DICT not in ARUCO_DICT:
        print(f"[ERROR] Dictionary '{DESIRED_DICT}' not found.")
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[DESIRED_DICT])

    for marker_id in range(NUM_MARKERS):
        marker_image = np.zeros((MARKER_SIZE_PIXELS, MARKER_SIZE_PIXELS), dtype=np.uint8)
        cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE_PIXELS, marker_image, 1)

        filename = os.path.join(SAVE_FOLDER, f"marker_{marker_id}.png")
        cv2.imwrite(filename, marker_image)
        print(f"[INFO] Saved marker ID {marker_id} to {filename}")

        # Display the marker briefly
        cv2.imshow(f"Marker {marker_id}", marker_image)
        cv2.waitKey(500)  # 0.5 second per marker
        cv2.destroyWindow(f"Marker {marker_id}")

if __name__ == "__main__":
    main()
