void initMotorController() {
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);

  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;
  if (spd < 0) {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;

  if (i == LEFT) {
    if (reverse == 0) {
      analogWrite(LEFT_MOTOR_FORWARD, spd);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    } else {
      analogWrite(LEFT_MOTOR_BACKWARD, spd);
      analogWrite(LEFT_MOTOR_FORWARD, 0);
    }
  } else {  // Right motor
    if (reverse == 0) {
      analogWrite(RIGHT_MOTOR_FORWARD, spd);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    } else {
      analogWrite(RIGHT_MOTOR_BACKWARD, spd);
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
    }
  }
}


void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  
  // Read encoder values
  long leftPosition = readEncoder(LEFT);
  long rightPosition = readEncoder(RIGHT);
  
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

