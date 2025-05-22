import numpy as np
import cv2
import time
import tensorflow as tf

# Load the TFLite model
interpreter = tf.lite.Interpreter(model_path="trainvschair_float32.tflite")
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Load and preprocess the image
image = cv2.imread("hoop.png")
input_shape = input_details[0]['shape'][1:3]  # e.g. (320, 320)

image_resized = cv2.resize(image, (input_shape[1], input_shape[0]))
image_input = image_resized.astype(np.float32) / 255.0  # normalize to [0,1]
image_input = np.expand_dims(image_input, axis=0)  # Add batch dimension

# # Depending on model input type, adjust dtype
# if input_details[0]['dtype'] == np.float32:
#     image_input = (image_input * 255).astype(np.float32)

# Set the tensor
interpreter.set_tensor(input_details[0]['index'], image_input)

# Inference timing
start_time = time.time()
interpreter.invoke()
end_time = time.time()

inference_time = (end_time - start_time) * 1000  # milliseconds
print(f"Inference time: {inference_time:.2f} ms")

# Get output tensor
output_data = interpreter.get_tensor(output_details[0]['index'])

# Post-process results (assumes output shape similar to ONNX example)
# Here: output_data shape might be (1, 2100, 5) or similar
output = np.squeeze(output_data, axis=0).T  # (2100, 5)


# print("Output shape:", output.shape)  # e.g. (5, 2100)
# print("Output example (first 10 values of each row):")
# for i, row in enumerate(output):
#     print(f"Row {i}:", row[:10])

# print(output.shape)

h_resized, w_resized, _ = image_resized.shape

detections = [det for det in output if det[4] > 0.35]

if detections:
    # Find detection with max confidence
    best_det = max(detections, key=lambda d: d[4])
    x, y, w, h, conf = best_det
    x_pixel = int(x * w_resized)
    y_pixel = int(y * h_resized)
    w_pixel = int(w * w_resized)
    h_pixel = int(h * h_resized)

    x1 = int(x_pixel - w_pixel / 2)
    y1 = int(y_pixel - h_pixel / 2)
    x2 = int(x_pixel + w_pixel / 2)
    y2 = int(y_pixel + h_pixel / 2)

    label = f"hoop: {conf:.2f}"
    cv2.rectangle(image_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(image_resized, label, (x1, max(y1 - 10, 0)), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
    print(label)

# Display inference time on image
cv2.putText(image_resized, f"Inference: {inference_time:.2f} ms", (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

cv2.imshow("Detections", image_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()
