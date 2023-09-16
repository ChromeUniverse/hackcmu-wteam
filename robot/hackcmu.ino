#include <arduino.h>

#define PIN_STANDBY 3
#define PIN_MOTOR_R_DIRECTION 7
#define PIN_MOTOR_L_DIRECTION 8
#define PIN_MOTOR_R_PWM 5
#define PIN_MOTOR_L_PWM 6

void setup() {
  pinMode(PIN_STANDBY, OUTPUT);
  pinMode(PIN_MOTOR_R_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_L_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);

  // turn on all motors
  digitalWrite(PIN_STANDBY, HIGH);

  Serial.begin(9600);

  Serial.setTimeout(10);
}

void loop() {

  // Serial.println()  
  
  if (Serial.available() > 0) {
    // read incoming stream
    String incomingByte = Serial.readString();

    int motor_r_direction = incomingByte.substring(0,3).toInt();
    int motor_l_direction = incomingByte.substring(3,6).toInt();
    int motor_r_speed = incomingByte.substring(6,9).toInt();
    int motor_l_speed = incomingByte.substring(9,12).toInt();

    // right motors

    // HIGH forwards, LOW backwards
    digitalWrite(PIN_MOTOR_R_DIRECTION, motor_r_direction);
    // 0 - stop, 255 - max speed
    analogWrite(PIN_MOTOR_R_PWM, motor_r_speed);

    // left motors

    // HIGH forwards, LOW backwards
    digitalWrite(PIN_MOTOR_L_DIRECTION, motor_l_direction);
    // 0 - stop, 255 - max speed
    analogWrite(PIN_MOTOR_L_PWM, motor_l_speed);
  }

}