import cv2
import numpy as np

# Initialize the camera (use 0 for the default camera, or adjust for other cameras)
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply thresholding to detect black regions
    _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)  # Adjust threshold as needed
    
    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Loop through contours and filter for rectangles
    for cnt in contours:
        # Approximate contour to reduce number of vertices
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        
        # Check if it has 4 vertices and is a closed shape
        if len(approx) == 4 and cv2.isContourConvex(approx):
            # Calculate the aspect ratio
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h

            # Filter based on aspect ratio and area
            if 0.8 < aspect_ratio < 1.2 and cv2.contourArea(cnt) > 1000:  # Adjust thresholds as needed
                # Draw a green rectangle around detected objects
                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)
    
    # Display the frame with detected rectangles
    cv2.imshow("Black Rectangle Detection", frame)
    
    # Press 'q' to break out of the loop and end the program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
