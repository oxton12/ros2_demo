import rclpy
from rclpy.node import Node
import sys
import glob
import yaml
from geometry_msgs.msg import Twist, Vector3
import math

from custom_interfaces.srv import Detection


class PositionEstimator(Node):

  def __init__(self):
    super().__init__('position_estimator')

    objects_info_path = glob.glob('./**/objects_info.yaml', recursive=True)[0]
    with open(objects_info_path, 'r') as file:
      objects_info = yaml.safe_load(file)
    self.widths = objects_info["widths"]
    self.focal_length = objects_info["focal_length"]

    self.declare_parameter("class_name", "cell_phone")
    self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    self.client = self.create_client(Detection, 'detection')
    while not self.client.wait_for_service(timeout_sec=1.0):
      self.get_logger().info('Waiting for stream to start ...')

    timer_period = 1/30 - 0.015
    self.timer = self.create_timer(timer_period, self.get_detection_callback)


  def get_detection_callback(self):
    request = Detection.Request()
    class_name = self.get_parameter('class_name').get_parameter_value().string_value
    request.class_name = class_name
    future = self.client.call_async(request)
    future.add_done_callback(self.publish_position_callback)
  

  def publish_position_callback(self, future):
    response = future.result()
    msg = self.get_twist(response)
    self.publisher.publish(msg)
  

  def get_twist(self, xywh):
    if xywh.x == -1 and xywh.y == -1: 
      return Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
    
    x, y, w, h = xywh.x, xywh.y, xywh.w, xywh.h
    class_name = self.get_parameter('class_name').get_parameter_value().string_value
    object_width = self.widths[class_name]
    distance = object_width * self.focal_length / w
    angular_y = math.atan((object_width / w * y) / distance)
    angular_z = math.atan((object_width / w * x) / distance)
    speed = distance - 0.3

    msg = Twist(linear=Vector3(x=speed, y=0.0, z=0.0), angular=Vector3(x=0.0, y=angular_y, z=angular_z))
    return msg


def main(args=None):
  rclpy.init(args=args)

  position_estimator = PositionEstimator()

  rclpy.spin(position_estimator)

  position_estimator.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()