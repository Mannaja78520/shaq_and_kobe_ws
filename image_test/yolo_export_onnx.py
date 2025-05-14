from ultralytics import YOLO

# Load the PyTorch model
model = YOLO("trainvschair.pt")

# Export to ONNX (specify opset and simplify to improve compatibility)
model.export(format="onnx", opset=12, simplify=True, dynamic=False)


