#include <Encoder.h>

// Create encoder objects
Encoder leftEncoder(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
Encoder rightEncoder(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
// Read encoder values
long leftPosition = leftEncoder.read();
long rightPosition = rightEncoder.read();

#ifdef USE_BASE
  #define ARDUINO_ENC_COUNTER

  #ifdef ARDUINO_ENC_COUNTER

    // Function to read encoder counts
    long readEncoder(int i) {
    
      long leftPosition = leftEncoder.read();
      long rightPosition = rightEncoder.read();
      // Apply 2:1 belt ratio (adjust for your setup)
      leftPosition /= 2;
      rightPosition /= -2;
      
      if (i == LEFT) return leftPosition;
      else return rightPosition;
    }

    // Function to reset encoder counts
    void resetEncoder(int i) {
      Serial.print("------------RESET------------");

      if (i == LEFT) {
        leftEncoder.write(0);
      } else {
        rightEncoder.write(0);
      }
    }

    void resetEncoders() {
      resetEncoder(LEFT);
      resetEncoder(RIGHT);
    }

  #else
    #error A encoder driver must be selected!
  #endif

#endif
