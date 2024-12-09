from ultralytics import YOLO
import torch

device = "cuda" if torch.cuda.is_available() else "cpu"
# Load a model
model = YOLO("yolo11n.pt").to(device)

# # Train the model
# train_results = model.train(
#     data="coco8.yaml",  # path to dataset YAML
#     epochs=100,  # number of training epochs
#     imgsz=640,  # training image size
#     device="cpu",  # device to run on, i.e. device=0 or device=0,1,2,3 or device=cpu
# )

# # Evaluate model performance on the validation set
# metrics = model.val()


# Perform object detection on an image
# results = model("OIP.jpg", conf=0.6)
results = model("download.jpg", conf=0.6)

results[0].show()

# Export the model to ONNX format
# path = model.export(format="onnx")  # return path to exported model