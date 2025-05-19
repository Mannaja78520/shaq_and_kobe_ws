from ultralytics import YOLO

# Load the PyTorch model
model = YOLO("trainvschair.pt")

# Export to ONNX (specify opset and simplify to improve compatibility)
model.export(format="onnx",
            #  format="TensorRT",
            opset=12,
            simplify=True,
            imgsz=(640, 480),
            optimize = False,
            # int8 = True,
            # dynamic = True,
            device = "cpu",
            # Specifies the device for exporting: GPU (device=0), CPU (device=cpu), MPS for Apple silicon (device=mps) or
            # DLA for NVIDIA Jetson (device=dla:0 or device=dla:1). TensorRT exports automatically use GPU.
            )


