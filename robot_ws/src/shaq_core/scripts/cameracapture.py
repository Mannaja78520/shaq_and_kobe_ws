#!/usr/bin/env python3

import cv2
import numpy as np

class CameraCapture:
    def __init__(self, camera_index=0, width=640, height=480):
        """
        camera_index: 0 = main camera
        width, height: Set lower resolution for better performance
        """
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce frame buffering
        self.cap.set(cv2.CAP_PROP_FPS, 20)  # Lower FPS to reduce CPU usage
        


        if not self.cap.isOpened():
            raise Exception("Cannot open camera")

    def get_screenshot(self):
        """ Capture image from camera """
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to capture image from camera")
        return np.ascontiguousarray(frame)

    def release(self):
        """ Close camera connection """
        self.cap.release()
