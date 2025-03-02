import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String


class SerialWriter(Node):

  def __init__(self):
    super().__init__('serial_writer')

    self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.02, parity=serial.PARITY_EVEN)

    self.subscription = self.create_subscription(String, 'cmd_actr', self.listener_callback, 10)
    self.subscription  # prevent unused variable warning


  def listener_callback(self, msg):
    cmd = msg.data
    self.arduino.write(cmd.encode())
    # self.get_logger().info(cmd)


def main(args=None):
  rclpy.init(args=args)
  serial_writer = SerialWriter()
  rclpy.spin(serial_writer)
  serial_writer.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
