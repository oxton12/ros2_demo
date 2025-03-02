import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MotorsDriver(Node):

  def __init__(self):
    super().__init__('motors_driver')

    self.forward_PID = PIDCntrlr(650, 300, 300)
    self.turn_PID = PIDCntrlr(700, 200, 500)

    self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
    self.subscription  # prevent unused variable warning

    self.publisher = self.create_publisher(String, 'cmd_actr', 10)
    msg = ("s120_90_")
    time.sleep(2)
    self.publisher.publish(String(data=msg))
    time.sleep(0.3)
    self.publisher.publish(String(data="d"))
    time.sleep(0.3)


  def listener_callback(self, msg):
    forward_cmd = msg.linear.x
    turn_cmd = msg.angular.z

    forward_speed = self.forward_PID(forward_cmd)
    turning_speed = self.turn_PID(turn_cmd)

    left_speed = forward_speed + turning_speed
    right_speed = forward_speed - turning_speed

    if left_speed >= 0 and right_speed >= 0:
      direction_code = 0
    elif left_speed >= 0 and right_speed < 0:
      direction_code = 1
    elif left_speed < 0 and right_speed < 0:
      direction_code = 2
    else:
      direction_code = 3

    left_speed = min(abs(left_speed), 100)
    right_speed = min(abs(right_speed), 100)

    command = ("m%d%d_%d_" % (direction_code, left_speed, right_speed))
    self.publisher.publish(String(data=command))



def main(args=None):
  rclpy.init(args=args)
  motors_driver = MotorsDriver()
  rclpy.spin(motors_driver)
  motors_driver.destroy_node()
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
    
    if(self.prev_value * value < 0):
      self.accumulation = 0

    if(value == 0):
      self.accumulation = 0
      self.prev_time = time.time()
      return 0

    current_time = time.time()
    
    time_diff = current_time - self.prev_time
    self.accumulation += time_diff * value
    value_diff = value - self.prev_value
    output = self.P * value + self.I * self.accumulation * value + self.D * (value_diff / time_diff) * value

    self.prev_time = current_time
    self.prev_value = value

    return int(output)