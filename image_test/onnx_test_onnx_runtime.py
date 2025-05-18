import onnxruntime as ort
import cv2
import numpy as np

# Load the ONNX model using onnxruntime
ort_session = ort.InferenceSession("trainvschair.onnx")

# Load and resize the image
image = cv2.imread("hoop2.png")
if image is None:
    raise ValueError("Image not found or invalid path!")

# Resize to model input size
input_size = (640, 640)
image_resized = cv2.resize(image, input_size)

# Preprocess the image for YOLO input
# Convert BGR to RGB
# input_image = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
# Transpose from HWC to CHW format
# Normalize to [0,1]
input_image = image_resized.astype(np.float32) / 255.0
# Add batch dimension
input_image = input_image.transpose(2, 0, 1)
input_image = np.expand_dims(input_image, axis=0)

# Get input and output names
input_name = ort_session.get_inputs()[0].name
output_names = [output.name for output in ort_session.get_outputs()]

# Run inference
outputs = ort_session.run(output_names, {input_name: input_image})

# Process YOLO outputs
# The format of outputs depends on your model, this is for YOLOv8
# Typically output[0] contains detection results with shape [1, num_detections, 7]
# where each detection has [x1, y1, x2, y2, confidence, class_id, class_score]
detections = outputs[0]

# Define class names (adjust according to your model)
class_names = {0: "chair"}  # Update with your actual class names

# Extract valid detections (confidence threshold)
confidence_threshold = 0.5
valid_detections = detections[0][detections[0][:, 4] > confidence_threshold]

# Draw bounding boxes
for detection in valid_detections:
    box = detection[:4]
    confidence = detection[4]
    class_id = int(detection[5])
    
    x1, y1, x2, y2 = map(int, box)
    
    # Draw rectangle
    cv2.rectangle(image_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    # Add label
    class_name = class_names.get(class_id, f"Class {class_id}")
    label = f"{class_name}: {confidence:.2f}"
    cv2.putText(image_resized, label, (x1, y1 - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

# Show the result
cv2.imshow("YOLO_Onnx_Detection", image_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()
