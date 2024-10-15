typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;

  int PrevInput;

  int ITerm;                    //integrated term

  long output;                    // last motor setting
}

SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  // Read encoder difference (taking into account the 2:1 ratio)
  input = 2 * (p->Encoder - p->PrevEnc);  // Multiply by 2 for the gear ratio
  Perror = p->TargetTicksPerFrame - input;

  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

const int adjustmentValue = 2; // ค่าที่ใช้ในการปรับเปลี่ยน
const long TOLERANCE = 50; // ค่าความแตกต่างที่อนุญาต
const long desiredSpeed = 100; // ความเร็วที่ต้องการ

/* Read the encoder values and call the PID routine */
void updatePID() {

  /* Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);
  
  long encoderDifference = leftPID.Encoder - rightPID.Encoder;

  if (abs(encoderDifference) > TOLERANCE) {
      if (encoderDifference > 0) {
          // ล้อซ้ายหมุนเร็วเกินไป
          if(leftPID.TargetTicksPerFrame < 0) {
            leftPID.TargetTicksPerFrame += adjustmentValue;
          }else{
            leftPID.TargetTicksPerFrame -= adjustmentValue;
          }
      } else {
          // ล้อขวาหมุนเร็วเกินไป
          if(rightPID.TargetTicksPerFrame < 0) {
            rightPID.TargetTicksPerFrame += adjustmentValue;
          }else{
            rightPID.TargetTicksPerFrame -= adjustmentValue;
          }
      }
  }
  if(leftPID.TargetTicksPerFrame < 0) {
    leftPID.TargetTicksPerFrame += adjustmentValue; // สำหรับล้อซ้าย
  }else{
    leftPID.TargetTicksPerFrame -= adjustmentValue;
  }
  //rightPID.TargetTicksPerFrame = desiredSpeed; // สำหรับล้อขวา
  
  /* If we're not moving there is nothing more to do */
  if (!moving){
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}

