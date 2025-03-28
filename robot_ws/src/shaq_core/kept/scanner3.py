import cv2
import numpy as np
import apriltag
from threading import Thread

class VideoStream:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 64)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 32)
        self.cap.set(cv2.CAP_PROP_FPS, 10)  # Limit FPS to 10

        self.ret, self.frame = self.cap.read()
        self.running = True
        Thread(target=self.update, daemon=True).start()

    def update(self):
        while self.running:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.running = False
        self.cap.release()

# AprilTag Detector Setup
options = apriltag.DetectorOptions(families="tag36h11", quad_decimate=2.0)
detector = apriltag.Detector(options)

# Start Video Stream
vs = VideoStream(0)
frame_skip = 2
frame_count = 0

while True:
    ret, frame = vs.read()
    if not ret:
        break

    frame_count += 1
    if frame_count % frame_skip != 0:
        continue  # Skip frames to improve FPS

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    for detection in detections:
        tag_id = detection.tag_id
        corners = detection.corners.astype(int)
        perceived_width = np.linalg.norm(corners[0] - corners[1])
        distance = (0.087 * 653) / perceived_width  # Adjust TAG_SIZE & FOCAL_LENGTH

        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)

        cX, cY = int(detection.center[0]), int(detection.center[1])
        cv2.putText(frame, f"ID: {tag_id}", (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, f"Dist: {distance:.2f}m", (cX - 20, cY + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    cv2.imshow("AprilTag Distance Estimation", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vs.stop()
cv2.destroyAllWindows()
