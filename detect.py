from ultralytics import YOLO

model = YOLO("best1.pt") #best2.pt (detect videoได้ดีกว่า) Precision=0.914
                         #best1.pt detcetรูปดีกว่า ''' เลือกใช้เอาเด้อจ้าาา '''

results = model.predict("test/001.jpg",conf = 0.6)
results[0].show()
