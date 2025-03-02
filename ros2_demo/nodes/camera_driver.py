import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CameraDriver(Node):

  def __init__(self):
    super().__init__('camera_driver')

    self.current_y_deg = 120
    self.current_z_deg = 90

    self.y_PID = PIDCntrlr(0.0, 0.0, 0.0)
    self.z_PID = PIDCntrlr(0.0, 0.0, 0.0)

    self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
    self.subscription  # prevent unused variable warning

    self.publisher = self.create_publisher(String, 'cmd_actr', 10)
    msg = ("s%d_%d_" % (self.current_y_deg, self.current_z_deg))
    time.sleep(2)
    self.publisher.publish(String(data=msg))
    time.sleep(0.3)
    self.publisher.publish(String(data="d"))
    time.sleep(0.3)


  def listener_callback(self, msg):
    if msg.angular.y == 0 and msg.angular.z == 0:
      self.y_PID(0)
      self.z_PID(0)
      self.publisher.publish(String(data="d"))
      return

    y_deg = msg.angular.y / 3.1415 * 180
    z_deg = msg.angular.z / 3.1415 * 180

    new_y_deg = self.y_PID(y_deg) + self.current_y_deg
    new_z_deg = self.z_PID(z_deg) + self.current_z_deg

    new_y_deg = max(0, min(180, new_y_deg))
    new_z_deg = max(0, min(180, new_z_deg))

    self.current_y_deg = new_y_deg
    self.current_z_deg = new_z_deg

    command = ("s%d_%d_" % (new_y_deg, new_z_deg))
    self.publisher.publish(String(data=command))



def main(args=None):
  rclpy.init(args=args)
  camera_driver = CameraDriver()
  rclpy.spin(camera_driver)
  camera_driver.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()


class PIDCntrlr:
  def __init__(self, P, I, D):
    self.P = P
    self.I = I
    self.D = D

    self.accumulation = 0
    self.prev_value = 0
    self.prev_time = 0

  def __call__(self, value):
    if(self.prev_time == 0): 
      self.prev_time = time.time()
      self.prev_value = value
      return 0
    
    if(self.prev_value * value < 0 or value == 0):
      self.accumulation = 0

    current_time = time.time()
    
    time_diff = current_time - self.prev_time
    self.accumulation += time_diff * value
    value_diff = value - self.prev_value
    output = self.P * value + self.I * self.accumulation * value + self.D * (value_diff / time_diff) * value

    self.prev_time = current_time
    self.prev_value = value

    output = min(2, max(-2, output))
    return int(output)