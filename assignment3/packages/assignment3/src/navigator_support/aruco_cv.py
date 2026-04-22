from __future__ import annotations

import cv2


def build_aruco_dictionary(name: str):
    # OpenCV exposes dictionaries as attributes like DICT_5X5_50; we look it up by string from the launch file.
    attr = getattr(cv2.aruco, name, None)
    if attr is None:
        raise ValueError("Unsupported ArUco dictionary '%s'" % name)
    return cv2.aruco.getPredefinedDictionary(attr)


def build_detector_parameters():
    # OpenCV 4.x uses DetectorParameters(); older API used DetectorParameters_create().
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def detect_markers(image, dictionary, parameters):
    # OpenCV 4.7+ prefers ArucoDetector; older code used detectMarkers directly.
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(image)
    return cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
