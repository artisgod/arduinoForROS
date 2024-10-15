#define USE_BASE 
/* Encoders directly attached to Arduino board */
#define ARDUINO_ENC_COUNTER
/* L298 Motor driver*/
#define L298_MOTOR_DRIVER

#define BAUDRATE 115200
#define MAX_PWM 255

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "commands.h"
#include "sensors.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"

#define PID_RATE           30

const int PID_INTERVAL = 1000 / PID_RATE;

unsigned long nextPID = PID_INTERVAL;

#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;

int arg = 0;
int index = 0;

char chr;
char cmd;
char argv1[16];
char argv2[16];

long arg1;
long arg2;

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK"); 
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK"); 
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case PING:
      Serial.println(Ping(arg1));
      break;
      
///////////////////////ENCODER////////////////////////////
      
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
      
///////////////////////MOTOR DRIVER///////////////////////
      
    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      }
      else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK"); 
      break;
      
    case MOTOR_RAW_PWM:
      lastMotorCommand = millis();
      resetPID();
      moving = 0;
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK"); 
      break;
      
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
         pid_args[i] = atoi(str);
         i++;
      }
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial.println("OK");
      break;
      
    default:
      Serial.println("Invalid Command");
      break;
  }
}

void setup() {
  Serial.begin(BAUDRATE);
  
  // Set encoder pins as input
  DDRD &= ~(1 << PD2);  // LEFT_ENC_PIN_A
  DDRD &= ~(1 << PD3);  // LEFT_ENC_PIN_B
  DDRD &= ~(1 << PC4);  // RIGHT_ENC_PIN_A
  DDRD &= ~(1 << PC7);  // RIGHT_ENC_PIN_B

  // Enable pull-up resistors
  PORTD |= (1 << PD2);
  PORTD |= (1 << PD3);
  PORTD |= (1 << PC4);
  PORTD |= (1 << PC7);

  // Enable pin change interrupts for encoder pins
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT23); // PD2, PD3, PD4, PD7

  initMotorController();
  resetPID();
  
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    char chr = Serial.read();

    if (chr == '\r') { // Carriage return indicates end of command
      // Null-terminate the arguments
      if (arg == 1) argv1[index] = '\0';
      else if (arg == 2) argv2[index] = '\0';

      // Execute the command
      runCommand();

      // Reset command variables
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1; // First argument
      else if (arg == 1) {
        argv1[index] = '\0'; // Null-terminate first argument
        arg = 2;             // Second argument
        index = 0;           // Reset index for second argument
      }
      // Continue to next character
    }
    else {
      if (arg == 0) {
        cmd = chr; // Store command
      }
      else if (arg == 1) {
        argv1[index] = chr; // Store first argument
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr; // Store second argument
        index++;
      }
    }
  }

  // Update PID at regular intervals
  unsigned long currentMillis = millis();
  if (currentMillis > nextPID) {
    updatePID();
    nextPID = currentMillis + PID_INTERVAL;
  }

  // Auto-stop motors if no command received within interval
  if ((currentMillis - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}

