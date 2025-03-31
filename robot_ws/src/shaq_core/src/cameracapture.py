import cv2
import numpy as np

class CameraCapture:
    def __init__(self, camera_index=0):
        """
        camera_index: ค่า index ของกล้อง (0 คือกล้องหลัก, 1 คือกล้องเสริม)
        """
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise Exception("Cannot open camera")

    def get_screenshot(self):
        """
        ถ่ายภาพจากกล้องและคืนค่าเป็น NumPy array (RGB format)
        """
        ret, frame = self.cap.read() 
        # retake
        if not ret:
            raise Exception("Failed to capture image from camera")

        # แปลงจาก BGR (ค่าเริ่มต้นของ OpenCV) เป็น RGB
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(frame)

    def release(self):
        """
        ปิดการเชื่อมต่อกับกล้อง
        """
        self.cap.release()

