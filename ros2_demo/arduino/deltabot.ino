#include <AFMotor.h>
#include <Servo.h>

typedef unsigned char uchar;

AF_DCMotor motor_rear_left(1);
AF_DCMotor motor_rear_right(2);
AF_DCMotor motor_front_right(3);
AF_DCMotor motor_front_left(4);

Servo camY;
Servo camZ;

bool attached = false;

void read_servo_cmd()
{
  if (!attached)
  {
    camY.attach(A0);
    camZ.attach(A1);
    attached = true;
  }

  int angles[2] = {0, 0};
  int angle_id = 0;
  uchar crc = 0xFF;

  while (angle_id < 2)
  {
    if (!Serial.available())
      continue;

    uchar input_byte = Serial.read();
    crc ^= input_byte;
    for (int j = 0; j < 8; ++j)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ 0x31;
      }
      else
      {
        crc <<= 1;
      }
    }

    if (input_byte == '_')
    {
      ++angle_id;
      continue;
    }
    angles[angle_id] = angles[angle_id] * 10 + int(input_byte) - 48;
  }
  crc = crc & 0xFF;

  while (!Serial.available())
  {
    continue;
  }
  uchar check_sum = Serial.read();
  if (check_sum != crc)
  {
    return;
  }

  camY.write(angles[0]);
  camZ.write(angles[1]);
}

void read_motor_cmd()
{
  int speeds[2] = {0, 0};
  int speed_id = 0;
  int direction_code = 0;
  bool first = true;
  uchar crc = 0xFF;

  while (speed_id < 2)
  {
    if (!Serial.available())
      continue;

    uchar input_byte = Serial.read();
    crc ^= input_byte;
    for (int j = 0; j < 8; ++j)
    {
      if (crc & 0x80)
      {
        crc = (crc << 1) ^ 0x31;
      }
      else
      {
        crc <<= 1;
      }
    }
    if (first)
    {
      direction_code = int(input_byte) - 48;
      first = false;
      continue;
    }

    if (input_byte == '_')
    {
      ++speed_id;
      continue;
    }
    speeds[speed_id] = speeds[speed_id] * 10 + int(input_byte) - 48;
  }
  crc = crc & 0xFF;

  while (!Serial.available())
  {
    continue;
  }
  uchar check_sum = Serial.read();
  if (check_sum != crc)
  {
    return;
  }

  motor_rear_left.setSpeed(speeds[0]);
  motor_rear_right.setSpeed(speeds[1]);
  motor_front_left.setSpeed(speeds[0]);
  motor_front_right.setSpeed(speeds[1]);

  switch (direction_code)
  {
  case 0:
    motor_rear_left.run(FORWARD);
    motor_front_left.run(FORWARD);
    motor_rear_right.run(FORWARD);
    motor_front_right.run(FORWARD);
    break;
  case 1:
    motor_rear_left.run(FORWARD);
    motor_front_left.run(FORWARD);
    motor_rear_right.run(BACKWARD);
    motor_front_right.run(BACKWARD);
    break;
  case 2:
    motor_rear_left.run(BACKWARD);
    motor_front_left.run(BACKWARD);
    motor_rear_right.run(BACKWARD);
    motor_front_right.run(BACKWARD);
    break;
  case 3:
    motor_rear_left.run(BACKWARD);
    motor_front_left.run(BACKWARD);
    motor_rear_right.run(FORWARD);
    motor_front_right.run(FORWARD);
    break;
  }
}

void setup()
{
  motor_rear_left.setSpeed(255);
  motor_rear_left.run(RELEASE);
  motor_rear_right.setSpeed(255);
  motor_rear_right.run(RELEASE);
  motor_front_right.setSpeed(255);
  motor_front_right.run(RELEASE);
  motor_front_left.setSpeed(255);
  motor_front_left.run(RELEASE);

  Serial.begin(9600);
}

void loop()
{
  if (Serial.available() > 0)
  {
    uchar cmd = Serial.read();
    switch (cmd)
    {
    case 'd':
      camY.detach();
      camZ.detach();
      attached = false;
      break;

    case 's':
      read_servo_cmd();
      break;

    case 'm':
      read_motor_cmd();
      break;

    default:
      break;
    }
  }
}
