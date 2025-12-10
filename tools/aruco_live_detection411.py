#!/usr/bin/env python

'''
Welcome to the ArUco Marker Detector!

This program:
  - Detects ArUco markers using OpenCV and Python
'''

from __future__ import print_function  # Python 2/3 compatibility
import cv2  # Import the OpenCV library
import numpy as np  # Import Numpy library

# Project: ArUco Marker Detector
# Date created: 12/18/2021
# Python version: 3.8
# Reference: https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

desired_aruco_dictionary = "DICT_6X6_250" # "DICT_ARUCO_ORIGINAL"

# The different ArUco dictionaries built into the OpenCV library.
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
    """
    Main method of the program.
    """
    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(
            args["type"]))
        sys.exit(0)
    print("OpenCV version: {}".format(cv2.__version__))
    # Load the ArUco dictionary
    print("[INFO] detecting '{}' markers...".format(
        desired_aruco_dictionary))
    this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
    this_aruco_parameters = cv2.aruco.DetectorParameters()

    # Start the video stream
    cap = cv2.VideoCapture(0)

    while (True):

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = cap.read()

        # Detect ArUco markers in the video frame

        # (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, this_aruco_dictionary)
        detector = cv2.aruco.ArucoDetector(this_aruco_dictionary, this_aruco_parameters)
        (corners, ids, rejected) = detector.detectMarkers(image=frame)
        cv2.aruco.drawDetectedMarkers(image=frame, corners=corners, ids=ids)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # If "q" is pressed on the keyboard,
        # exit this loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Close down the video stream
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print(__doc__)
    main()
