from ultralytics import YOLO
import cv2

class ObjectDetector:
    def __init__(self, model_path="trained_gate.pt"):
        self.model = YOLO(model_path).to("cpu")
        self.class_names = self.model.names
    
    def detect_objects(self, frame, threshold=0.5, imgsz=320):
        results = self.model(frame, imgsz=imgsz, verbose=False, conf=threshold)
        
        detections = []
        
        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    if conf >= threshold:
                        label = self.class_names[cls]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        detections.append((x1, y1, x2, y2, label, conf))
        
        return detections   
