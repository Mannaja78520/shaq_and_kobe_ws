import cv2
import numpy as np
import tensorflow as tf  # or tflite_runtime.interpreter as tflite

# Load TFLite model and allocate tensors
interpreter = tf.lite.Interpreter(model_path="trainvschair_float32.tflite")
interpreter.allocate_tensors()

# Get input and output tensor details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Load and preprocess image
image = cv2.imread("hoop.png")
if image is None:
    raise ValueError("Image not found or invalid path!")

input_size = (input_details[0]['shape'][2], input_details[0]['shape'][1])  # (width, height)
image_resized = cv2.resize(image, input_size)
# input_tensor = np.expand_dims(image_resized, axis=0).astype(np.uint8)  # or np.float32 depending on model
# image_resized = image_resized.transpose(1, 0, 2)
input_tensor = (np.expand_dims(image_resized, axis=0).astype(np.float32)) / 255.0


# Run inference
interpreter.set_tensor(input_details[0]['index'], input_tensor)
interpreter.invoke()

input_details = interpreter.get_input_details()
print("Expected input shape:", input_details[0]['shape'])
print("Expected input dtype:", input_details[0]['dtype'])


# # Postprocess output
# # Output format depends on model. Here’s a general example for YOLO-style output
# output_data = interpreter.get_tensor(output_details[0]['index'])

# # Placeholder parsing — you need to adjust based on model
# # Example assumes boxes in output_data: [batch, num_boxes, 6] (x1, y1, x2, y2, score, class_id)
# for det in output_data[0]:  
#     x1, y1, x2, y2, conf, cls_id = map(int, det[:6])
#     if conf > 0.5:
#         cv2.rectangle(image_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
#         label = f"Class {cls_id}: {conf:.2f}"
#         cv2.putText(image_resized, label, (x1, y1 - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

# # Show result
# cv2.imshow("TFLite Detection", image_resized)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
