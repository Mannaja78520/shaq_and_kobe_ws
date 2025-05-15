from ultralytics import YOLO
import os

# Load the YOLO11 model
username = os.getenv("USER")
model_path = f"trainvschair.pt"
model = YOLO(model_path)
picture_path = f"hoop.png"

# Export the model to TFLite format
model.export(format="tflite")

# # Load the exported TFLite model
# tflite_model = YOLO("trainvschair_float32.tflite")

# # Run inference
# results = tflite_model(picture_path)