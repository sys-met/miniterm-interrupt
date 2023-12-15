/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   

  
#ifdef INTERRUPT_ENCODER_COUNTER

  #define LEFT_ENC_A 18
  #define LEFT_ENC_B 19
  #define RIGHT_ENC_A 21
  #define RIGHT_ENC_B 20
  
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
