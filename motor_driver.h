#define LEFT_MOTOR_BACKWARD  10
#define LEFT_MOTOR_FORWARD   6
#define LEFT_MOTOR_ENABLE    13

#define RIGHT_MOTOR_BACKWARD 9
#define RIGHT_MOTOR_FORWARD  5
#define RIGHT_MOTOR_ENABLE   12

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
