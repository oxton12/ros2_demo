import rclpy
from rclpy.node import Node
import cv2
import glob
import yaml
import numpy as np
from ultralytics import YOLO

from custom_interfaces.srv import Detection


class YoloVision(Node):

  def __init__(self):
    super().__init__('yolo_vision')

    model_path = glob.glob('./**/model/yolov10n.engine', recursive=True)[0]
    self.model = YOLO(model_path, task="detect")

    objects_info_path = glob.glob('./**/objects_info.yaml', recursive=True)[0]
    with open(objects_info_path, 'r') as file:
      objects_info = yaml.safe_load(file)
    self.classes = objects_info["classes"]
    self.colors = objects_info["colors"]

    self.cap = cv2.VideoCapture("rtsp://192.168.0.102:8554/cam")
    w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    self.fps = self.cap.get(cv2.CAP_PROP_FPS)

    calibration_path = glob.glob('./**/cam_calibration/calibration.npz', recursive=True)[0]
    camera_calibration = np.load(calibration_path)
    mtx = camera_calibration["mtx"]
    dist = camera_calibration["dist"]
    newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    self.mapx, self.mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
    self.x, self.y, self.w, self.h = self.roi

    self.yolo_first_run()

    self.srv = self.create_service(Detection, 'detection', self.detect_object_callback)


  def yolo_first_run(self):
    ret, img = self.cap.read()
    img = cv2.flip(img, 0)
    img = cv2.remap(img, self.mapx, self.mapy, cv2.INTER_LINEAR)
    img = img[self.y:self.y+self.h, self.x:self.x+self.w] 
    self.model.predict(img, device="cuda:0", half=True, verbose=False, classes=self.classes["donut"], max_det=1)


  def detect_object_callback(self, request, response):
    class_name = request.class_name

    ret, img = self.cap.read()
    img = cv2.flip(img, 0)
    img = cv2.remap(img, self.mapx, self.mapy, cv2.INTER_LINEAR)
    img = img[self.y:self.y+self.h, self.x:self.x+self.w]
    img = cv2.flip(img, 1)

    results = self.model.predict(img, device="cuda:0", half=True, verbose=False, classes=self.classes[class_name], max_det=1, conf=0.05)
    out = results[0].boxes

    box_xywh = [-1, -1, -1, -1]
    if out.cls.shape[0] >= 1 :
      cls = int(out.cls[0].item())
      name = results[0].names[cls]
      box = out.xyxy[0].int().tolist()
      box_xywh = out.xywh[0].int().tolist()
      cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), self.colors[class_name], 2)
      cv2.putText(img, name, (box[0] + 5, box[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, self.colors[class_name], 2)
      box_xywh[0] -= img.shape[1] // 2
      box_xywh[1] -= img.shape[0] // 2
        
    cv2.imshow("img", img)
    cv2.waitKey(1)

    response.x, response.y, response.w, response.h = box_xywh

    return response
  

def main(args=None):
  rclpy.init(args=args)
  position_estimator = YoloVision()
  rclpy.spin(position_estimator)
  position_estimator.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()