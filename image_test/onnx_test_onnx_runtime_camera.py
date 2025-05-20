import onnxruntime as ort
import numpy as np
import cv2
import time

# Load the ONNX model
session = ort.InferenceSession("trainvschair.onnx", providers=["CPUExecutionProvider"])

input_name = session.get_inputs()[0].name
output_names = [output.name for output in session.get_outputs()]
input_shape = (256, 256)  # Model input size

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Prepare input for model
    image_resized = cv2.resize(frame, input_shape)
    image_input = image_resized.transpose(2, 0, 1).astype(np.float32) / 255.0
    image_input = np.expand_dims(image_input, axis=0)

    start_time = time.time()
    outputs = session.run(output_names, {input_name: image_input})
    inference_time = (time.time() - start_time) * 1000  # ms

    output = outputs[0]
    output = np.squeeze(output, axis=0).T  # (N,5)

    # Draw on original frame but scale bbox to original frame size
    h_resized, w_resized, _ = image_resized.shape

    detections = [det for det in output if det[4] > 0.35]
    
    if detections:
        # Find detection with max confidence
        best_det = max(detections, key=lambda d: d[4])
        x, y, w, h, conf = best_det

        # Convert normalized center-based bbox to original frame coordinates
        x1 = int((x - w / 2) * w_resized)
        y1 = int((y - h / 2) * h_resized)
        x2 = int((x + w / 2) * w_resized)
        y2 = int((y + h / 2) * h_resized)

        cv2.rectangle(image_resized, (x1, y1), (x2, y2), (255,255,30), 2)
        label = f"hoop: {conf:.2f}"
        cv2.putText(image_resized, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,30), 1)
        # Show inference time on frame
        cv2.putText(image_resized, f"Inference: {inference_time:.1f} ms", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    cv2.imshow("Webcam Detections", image_resized)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
