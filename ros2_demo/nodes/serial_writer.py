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


  def crc8(self, data):
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
    return crc & 0xFF


  def listener_callback(self, msg):
    cmd = msg.data
    byte_cmd = cmd.encode()
    crc_value = self.crc8(byte_cmd)
    cmd_with_crc = byte_cmd + bytes([crc_value])
    self.arduino.write(byte_cmd)
    # self.get_logger().info(cmd)


def main(args=None):
  rclpy.init(args=args)
  serial_writer = SerialWriter()
  rclpy.spin(serial_writer)
  serial_writer.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
