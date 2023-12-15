#include "DualG2HighPowerMotorShield.h"
  
DualG2HighPowerMotorShield18v18 drive;
  /* Wrap the drive motor set speed function */



  // A convenience function for setting both motor speeds

#define LEFT_ENC_A 18
#define LEFT_ENC_B 19
#define RIGHT_ENC_A 21
#define RIGHT_ENC_B 20

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"




#include <util/atomic.h>
  /* Motor driver function definitions */
//#include "motor_driver.h"

  /* PID parameters and functions */
//#include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
#define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
#define AUTO_STOP_INTERVAL 4000
long lastMotorCommand = AUTO_STOP_INTERVAL;


/* Variable initialization */
volatile int counterLeft = 0;
volatile int counterRight = 0;
float ltick_target = 0;
float rtick_target = 0;
float lITerm = 0;
float rITerm = 0;
int leftPrevC = 0;
int rightPrevC = 0;
float lPrevInput = 0;
float rPrevInput = 0;
bool moved = false;
unsigned char moving = 0;
long loutput;
long routput;
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
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
  /*case PING:
    Serial.println(Ping(arg1));
    break;*/
    

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
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
      moved = false;
    }
    else moving = 1;
    //move_function(arg1, arg2);
    ltick_target = arg1;
    rtick_target = arg2;
    moved = true;
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    moved = false;
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  //case UPDATE_PID:
  //  while ((str = strtok_r(p, ":", &p)) != '\0') {
  //     pid_args[i] = atoi(str);
  //     i++;
  //  }
  //  Kp = pid_args[0];
  //  Kd = pid_args[1];
  //  Ki = pid_args[2];
  //  Ko = pid_args[3];
  //  Serial.println("OK");
  //  break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
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
//long readLEncoder(){
//  int left_enc_pos = 0;
//  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){left_enc_pos = counterLeft;}
//  return left_enc_pos;
//  }
long readEncoder(int i) {
    int left_enc_pos = 0;
    int right_enc_pos = 0;
    //ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    //  left_enc_pos = counterLeft;
    //  right_enc_pos = counterRight;
    //}  
    if (i == LEFT){
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){left_enc_pos = counterLeft;}
      return left_enc_pos;
    }

    //if (i == RIGHT) return right_enc_pos;
    else {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE){right_enc_pos = counterRight;}
      return right_enc_pos;
    }
  }

void initMotorController() {
  drive.init();
  drive.enableDrivers();
}
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM2Speed(spd);
  else drive.setM1Speed(spd);
}
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  //drive.setM1Speed(rightSpeed);
  //drive.setM2Speed(leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
void resetPID(){

   ltick_target = 0.0;
   rtick_target = 0.0;
   int countLeft = readEncoder(LEFT);
   int countRight = readEncoder(RIGHT);
   leftPrevC = countLeft;
   rightPrevC = countRight;
   loutput = 0;
   routput = 0;
   lPrevInput = 0;
   rPrevInput = 0;
   lITerm = 0;
   rITerm = 0;

}

void move_function(float ltick, float rtick) {
  int left_count = 0;
  int right_count = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    left_count = counterLeft;
    right_count = counterRight;
  }
  long lPerror;
  long rPerror;

  int linput;
  int rinput;
  float Kp = 28;
  float Ki = 0;
  float Kd = 0;
  float Ko = 50;
  //float ltick_target =ltick;//100*(sin(currT/1e6)>0);
  //float rtick_target =rtick;// 100*(sin(currT/1e6)>0);
  linput = left_count - leftPrevC;
  rinput = right_count - rightPrevC;
  
  lPerror = ltick - linput;
  rPerror = rtick - rinput;
  
  loutput = (Kp * lPerror - Kd * (linput - lPrevInput) + lITerm) / Ko;
  routput = (Kp * rPerror - Kd * (rinput - rPrevInput) + rITerm) / Ko;
  
  leftPrevC = left_count;
  rightPrevC = right_count;
  
  loutput += loutput;
  routput += routput;
  
  lITerm += Ki*lPerror;
  rITerm += Ki*rPerror;
  
  lPrevInput = linput;
  rPrevInput = rinput;
  
  setMotorSpeeds(loutput, routput);
  
}
/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);


  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), readRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), readLeftEncoder, RISING);

  initMotorController();

}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
// If we are using base control, run a PID calculation at the appropriate intervals

  if (millis() > nextPID && moved == true) {
    move_function(ltick_target, rtick_target);
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
    moved = false;
  }


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
