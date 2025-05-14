from ultralytics import YOLO
import cv2
import matplotlib.pyplot as plt

# Load the ONNX model
model = YOLO("trainvschair.onnx")

# Run inference on an image
results = model("hoop.png")[0]  # take the first result from the batch

# Load the image using OpenCV
image = cv2.imread("hoop.png")
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for matplotlib

# Draw bounding boxes
for box in results.boxes:
    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
    conf = float(box.conf[0])              # Confidence score
    cls_id = int(box.cls[0])               # Class ID
    cls_name = model.names[cls_id]         # Class name

    # Draw rectangle and label
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    label = f"{cls_name}: {conf:.2f}"
    cv2.putText(image, label, (x1, y1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

# Show image
plt.figure(figsize=(8, 6))
plt.imshow(image)
plt.axis("off")
plt.title("YOLO Detection with Confidence")
plt.show()
