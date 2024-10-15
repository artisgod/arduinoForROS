#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

// Left encoder pins on PORTD
#define LEFT_ENC_PIN_A PD2 
#define LEFT_ENC_PIN_B PD3

// Right encoder pins on PORTD
#define RIGHT_ENC_PIN_A 18
#define RIGHT_ENC_PIN_B 19
  
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

#endif  // ENCODER_DRIVER_H

