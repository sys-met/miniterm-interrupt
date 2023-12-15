/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }


#elif defined(INTERRUPT_ENCODER_COUNTER)
  volatile int counterLeft = 0;
  volatile int counterRight = 0;

  
  void readLeftEncoder(){
  
    int b = digitalRead(LEFT_ENC_B);
    int increment = 0;
  
    if (b > 0){
     increment = 1;
    }
    else{
      increment = -1;
    }
  
    counterLeft = counterLeft + increment;
  }

  
  void readRightEncoder(){
  
    int b = digitalRead(RIGHT_ENC_B);
    int increment = 0;
  
    if (b > 0) {
      increment = 1;
    }
    else {
      increment = -1;
    }
  
    counterRight = counterRight + increment; 
  }

  long readEncoder(int i) {
    int left_enc_pos = 0;
    int right_enc_pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      left_enc_pos = counterLeft;
      right_enc_pos = counterRight;
    }  
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  void resetEncoder(int i) {
 
    if (i == LEFT){
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){counterLeft = 0;}
      return;
    } else { 
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){counterRight = 0;}
      return;
    }
  }

#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
