import cv2
import numpy as np
import apriltag

# Define known tag size (meters or centimeters)
TAG_SIZE = 0.087  # Example: 10 cm tag

# Define focal length (use calibration to get this value)
FOCAL_LENGTH = 653  # Change this after calibration

# Initialize AprilTag detector
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

# Start video capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    detections = detector.detect(gray)

    for detection in detections:
        tag_id = detection.tag_id
        corners = detection.corners.astype(int)

        # Compute perceived width and height
        perceived_width = np.linalg.norm(corners[0] - corners[1])
        perceived_height = np.linalg.norm(corners[1] - corners[2])

        # Use the average size to estimate distance
        perceived_size = (perceived_width + perceived_height) / 2
        distance = (TAG_SIZE * FOCAL_LENGTH) / perceived_size
        


        # Calculate distance
        # distance = (TAG_SIZE * FOCAL_LENGTH) / perceived_width
        
        print(f"distance: {distance}")

        # Draw bounding box
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        # Display Tag ID and Distance
        cX, cY = int(detection.center[0]), int(detection.center[1])
        cv2.putText(frame, f"ID: {tag_id}", (cX - 20, cY - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, f"Dist: {distance:.2f}m", (cX - 20, cY + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # # Show the video feed
    cv2.imshow("AprilTag Distance Estimation", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
