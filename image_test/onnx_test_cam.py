from ultralytics import YOLO
import cv2

# Load the ONNX model
model = YOLO("trainvschair.onnx", task="detect")  # explicitly define task

# Open webcam (0 is the default camera)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise IOError("Cannot open webcam")

input_size = (384, 384)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Resize the frame
    frame_resized = cv2.resize(frame, input_size)

    # Run detection
    results = model.predict(source=frame_resized, imgsz=input_size)[0]

    # Draw bounding boxes
    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        cls_name = model.names[cls_id]

        cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{cls_name}: {conf:.2f}"
        cv2.putText(frame_resized, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Display the result
    cv2.imshow("YOLO_Onnx_Camera", frame_resized)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
