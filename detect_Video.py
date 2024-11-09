from ultralytics import YOLO

model = YOLO("best2.pt")

results = model.predict(source="test/010.mp4", show = True, conf=0.4, line_width = 2, save_crop = False, save_txt = False, show_labels = False, classes = [0,1])