import numpy as np
import cv2
import time
import tensorflow as tf

# Load the TFLite model
interpreter = tf.lite.Interpreter(model_path="trainvschair_float32.tflite")
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape'][1:3]  # e.g. (height, width)

cap = cv2.VideoCapture(0)  # Open default camera

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Preprocess frame
    image_resized = cv2.resize(frame, (input_shape[1], input_shape[0]))
    image_input = image_resized.astype(np.float32) / 255.0
    image_input = np.expand_dims(image_input, axis=0)

    interpreter.set_tensor(input_details[0]['index'], image_input)

    start_time = time.time()
    interpreter.invoke()
    inference_time = (time.time() - start_time) * 1000

    output_data = interpreter.get_tensor(output_details[0]['index'])
    output = np.squeeze(output_data, axis=0).T  # adapt if output shape is different

    h_resized, w_resized, _ = image_resized.shape
    detections = [det for det in output if det[4] > 0.35]

    # Draw detections
    for det in detections:
        best_det = max(detections, key=lambda d: d[4])
        x, y, w, h, conf = det
        x_pixel = int(x * w_resized)
        y_pixel = int(y * h_resized)
        w_pixel = int(w * w_resized)
        h_pixel = int(h * h_resized)

        x1 = int(x_pixel - w_pixel / 2)
        y1 = int(y_pixel - h_pixel / 2)
        x2 = int(x_pixel + w_pixel / 2)
        y2 = int(y_pixel + h_pixel / 2)

        cv2.rectangle(image_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"hoop: {conf:.2f}"
        cv2.putText(image_resized, label, (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display inference time on frame
    cv2.putText(image_resized, f"Inference: {inference_time:.2f} ms", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    cv2.imshow('Camera Detections', image_resized)

    if cv2.waitKey(1) & 0xFF == 27:  # Press ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
