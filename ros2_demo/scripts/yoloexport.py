from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("model/yolov10n.pt")

# Export the model to TensorRT format
model.export(format="engine", opset=17, simplify=True)  # creates 'yolo11n.engine'