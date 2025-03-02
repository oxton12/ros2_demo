#include <AFMotor.h>
#include <Servo.h>

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
  int value_id = 0;
  while (value_id < 2)
  {
    if (Serial.available())
    {
      char input_byte = Serial.read();
      Serial.println(input_byte);
      if (input_byte == '_')
      {
        ++value_id;
        continue;
      }
      angles[value_id] = angles[value_id] * 10 + int(input_byte) - 48;
      input_byte = Serial.read();
    }
  }
  camY.write(angles[0]);
  camZ.write(angles[1]);
  Serial.println(angles[0]);
  Serial.println(angles[1]);
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
    char cmd = Serial.read();
    Serial.println(cmd);
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

    default:
      break;
    }
  }
}
