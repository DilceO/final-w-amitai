#include <Arduino.h>
#include <RBE1001Lib.h>
#include "BlueMotor.h"
#include "Chassis.h"
#include "ESP32Servo.h"
#include "Timer.h"
#include "USAverager.h"
#include <IRdecoder.h>
#include <RemoteConstants.h>
#include "RobotConstants.h"

// BLUE MOTOR SET UP
BlueMotor blueMotor;
ESP32AnalogRead leftLine;
ESP32AnalogRead rightLine;

// THINGS FOR PRINTING PURPOSES
long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270;

// ARM TESTING
int motorEffort = -1;
int bringDown = -250;
int bringUp = 250;
float encoderCount = 0.0;
float changed = 0.0;

// //ARM SETTINGS FOR FUNCTIONS - TO BE TESTED 
// int roofSet45 = 100;
// int roofLift45 = 200;
// int blockArmSet = -100; 
// int roofSet25 = 150; 
// int roofLift25 = 230; 
// int restingSet = 40;

// PID VARIABLES
float PIDEffort = 0.0;
float setScaledEffort = 0.0;
const float Kp = 0.35;
const float Ki = 0.000;
const float Kd = 0.00;
float previousCount = 0.0;
float sumOfErrors = 0.0;
float Fourty5DegreeEncoderCount = 3500.0;
float Twenty5DegreeEncoderCount = 7700.0;
float stagingBlock = 0.0;
Rangefinder ultrasonic;
const float ultraKp = 0.0085;

// ROOF & BLOCK THRESHOLDS
int roofApproachThreshold = 18; // how far we want to be from roof to lift arm safely
int roofThreshold = 12;         // how far we want to be from roof to actually pick up/drop off plate
int blockThreshold = 10;        // how far we want to be from block to drop off/pick up

//BUTTON
const int buttonPin = BOOT_FLAG_PIN;

//MOTOR SET UP FOR DRIVING
LeftMotor left_motor;
RightMotor right_motor;
double diam = 2.75;
double track = 5.875;
int defaultSpeed = 130;
double distanceCorrection = 0.95;
const float pivot_diam = 14.2;
const float wheel_diam = 6.8;
const float degreesPerMotor = (pivot_diam * 3.14) / (wheel_diam * 3.14);
const float degreesPerCM = 360.0 / (3.14 * wheel_diam); // cm to degrees formula

// LINE FOLLOWING SET UP - CALLING THE LINE FOLLOWING SENSOR FUNCTION
const int reflectancePin1 = 39;
const int reflectancePin2 = 36;
int threshold = 0.5;
int thresholdHigh = 1250;
double kp = 0.05;
float leftV;
float rightV;

//SERVO ARM SET UP
Servo grip;
const int servoPin = 33;
int openGrip = 140; // find number which make it close
int closeGrip = 60; // find numbers which make it open

//IR REMOTE SET UP
IRDecoder decoder(15);
int16_t keyPress;

// BOOLEAN VARIABLES FOR PICK UP AND DROP OFF 
bool safeToBeCollected = false; // picking up old collector
bool safeToBeDeposited = false; // depositing old collector
bool safeToPickUpNew = false;   // picking up new collector
bool safeToDropOffNew = false;  // depositing new collector
bool readyToPickUpNew = false;  // once new plate has been placed on block by person
bool depositingNew = false; // false when we pick up old and deposit old, true when we have picked up new plate
int previousState = 0;

//STATE MACHINES
enum ROBOT_STATES
{
  APPROACH_ROOF_45,
  GRAB_PANEL_45,
  PLACE_BLOCK,
  OPPOSITE_SIDE,
  APPROACH_ROOF_25, 
  GRAB_PANEL_25,
  APPROACH_BLOCK,
  IDLE
};

int robotState;

//LIST OF NEW FUNCTIONS functions (need to check each of these once sensors are wired)
void turn(double angle);
void straight(double distance);
void openGripper ();
void closeGripper();
void armRaise45();
void liftOff45 ();
void lowerArm();
void lowerArmBlock();
void armRaise25();
void liftOff25();


// INITIAL SET UP FOR ROBOT STATE
void setup()
{
  Serial.begin(9600);
  Serial.println("setup started");
  grip.attach(servoPin);

  blueMotor.setup();

  decoder.init();
  blueMotor.reset();
  robotState = IDLE;
  delay(1000);
  Serial.println("setup complete");
}

// TIMER FUNCTION timer which is set to 1 / 100th of a second or 10 milliseconds
Timer PIDTimer(10);
float output;

// PID CONTROL PROGRAM
float evaluate(long encoderCount, long targetCount)
{
  if (PIDTimer.isExpired())
  {
    printf("enc: %li\n", encoderCount);
    float error = targetCount - encoderCount;
    sumOfErrors += error;
    output = Kp * error - Kd * (encoderCount - previousCount) + Ki * sumOfErrors;
    //printf("Current Encoder Count:%f\t Set Encoder Count:%f\t Output:%f\t", encoderCount, previousCount, output);
    previousCount = encoderCount;
  }
  return output;
}

// FIND ENCODER COUNT FUNCTION (Logging the position as the motor spins)
void findEncoderCount()
{
  blueMotor.setEffort(-100);
  long answer = blueMotor.getPosition();
  printf("count:%ld\n", answer);
}



//QUICK FUNCTIONS FOR BRINGING THE ARM DOWN AND UP
void bringArmDown()
{
  blueMotor.setEffort(evaluate(blueMotor.getPosition(),-6000));
}

void bringArmUp(){
  blueMotor.setEffort(evaluate(blueMotor.getPosition(),2580));
}

// NEW FUNCTIONS NEW FUNCTIONS NEW FUNCTIONS NEW FUNCTIONS NEW FUNCTIONS NEW FUNCTIONS NEW FUNCTIONS


  // // //TO CHECK VALUES IN THE AM 
  // void armValueCheck()
  // {
  //   blueMotor.setEffort(evaluate(blueMotor.getPosition(),5500));
  // }

//TURN FUNCTION DEFINITION
void turn(double degrees)
{
  left_motor.startMoveFor(-(degrees * degreesPerMotor), 180);
  right_motor.moveFor(degrees * degreesPerMotor, 180);
  delay(1000);
}

// STRAIGHT FUNCTION DEFINITION
void straight(double cm)
{
  left_motor.startMoveFor(cm * degreesPerCM, 90);
  right_motor.moveFor(cm * degreesPerCM, 90);
}

// CLOSE GRIPPER FUNCTION
void closeGripper()
{
  grip.write(closeGrip);
}

//OPEN GRIPPER FUNCTION
void openGripper()
{
  grip.write(openGrip);
}

//ARM RAISE FOR 45 FUNCTION
void armRaise45()
{
blueMotor.setEffort(evaluate(blueMotor.getPosition(),2000));
}

//LIFT OFF 45 FUNCTION
void liftOff45()
{
  blueMotor.setEffort(evaluate(blueMotor.getPosition(),1600));
}

// LOWER ARM FUNCTION
void lowerArm()
{
  blueMotor.setEffort(evaluate(blueMotor.getPosition(), -600));
}

//LOWER ARM BLOCK FUNCTION
void lowerArmBlock()
{
  blueMotor.setEffort(evaluate(blueMotor.getPosition(), -1000));
}

//ARM RAISE 25 FUNCTION 
void armRaise25()
{
  blueMotor.setEffort(evaluate(blueMotor.getPosition(), 7600));
}

// LIFT OFF 25 FUNCTION
void liftOff25()
{
  blueMotor.setEffort(evaluate(blueMotor.getPosition(), 2000));
}





//ONE OF THE OLD FUNCTIONS - WILL NEED TO BE DELETED LATER, KEPT FOR REFERENCE
// void pickUpOld()
// {
//   if (safeToBeCollected)
//   {
//     straight(0.5);

//     while (blueMotor.getPosition() < Fourty5DegreeEncoderCount + 4000)
//     {
//       //printf("reached here, lifting arm more\n");
//       PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount + 4000);
//       setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
//     }

//     grip.write(closeGrip);

//     while (blueMotor.getPosition() < Fourty5DegreeEncoderCount + 4000)
//     {
//       //printf("reached here, lifting arm more\n");
//       PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount + 4000);
//       setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
//     }

//     straight(1.25);

//     while (blueMotor.getPosition() > Fourty5DegreeEncoderCount - 12.5)
//     {
//       //printf("reached here, lifting arm more\n");
//       PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 12.5);
//       setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
//     }

//     printf("pickup position: %ld\n", blueMotor.getPosition());

//     straight(2);
//     blueMotor.setEffort(0);

//     while (blueMotor.getPosition() > Fourty5DegreeEncoderCount - 425)
//     {
//       //printf("reached here, lifting arm more\n");
//       PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 425);
//       setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
//       blueMotor.setEffort(0);
//     }
//     // back away from roof small amount
//     straight(-3); // have this go backwards

//     //forward just a sec once more hoe
//     //straight(1);

//     // while (blueMotor.getPosition() > Fourty5DegreeEncoderCount - 425)
//     // {
//     //   //printf("reached here, lifting arm more\n");
//     //   PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 425);
//     //   setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
//     // }
//     straight(-7);
//   }
//   else
//   {
//     printf("Swanbot is waiting to safely collect old plate\n");
//   }
// }


// STATE MACHINES START STATE MACHINES STATE MACHINES STATE MACHINES STATE MACHINES
void updateRobotState()
{
  leftV = leftLine.readVoltage();
  rightV = rightLine.readVoltage();

  switch (robotState)
  {
    if (keyPress == remotePlayPause)
    {
      previousState = robotState;
      robotState = IDLE;
    }
  case IDLE:
    printf("entered Idle\n");
    left_motor.setEffort(0);
    right_motor.setEffort(0);
    blueMotor.setEffort(0);

    // REMOTE BUTTON #9 - APPROACH ROOF 45 
    if (keyPress == remote9)
    {
      robotState = APPROACH_ROOF_45;
    }

  // REMOTE BUTTON #2 - GRAB PANEL 
    if (keyPress == remote1)
    {
      robotState == GRAB_PANEL_45;
    }
  
   // REMOTE BUTTON #3 - APPROACH BLOCK 
    if (keyPress == remote3)
    {
      robotState = APPROACH_BLOCK;
    }
   
   // REMOTE BUTTON 2 - PLACE BLOCK 
    if (keyPress == remote2){ 
      robotState == PLACE_BLOCK; 
    }
    
    // REMOTE BUTTON #4 - GO TO OPPOSITE SIDE
    if (keyPress == remote4)
    {
      robotState == OPPOSITE_SIDE;
    }
    //REMOTE BUTTON 5 = APPROACH ROOF 25 
    if (keyPress == remote5)
    {
      robotState = APPROACH_ROOF_25; 
    }

    //REMOTE BUTTON 6 - GRAB PANEL 25 
    if (keyPress == remote6)
    {
      robotState == GRAB_PANEL_25;
    }
    else
    {
      left_motor.setEffort(0);
      right_motor.setEffort(0);
      blueMotor.setEffort(0);
    }
      break; 
    
    case APPROACH_ROOF_45: 
      straight(20);
      delay(2000);
      openGripper(); 
      delay(2000); 
      armRaise45(); 
      delay(2000);
      //straight(0.5);
      delay(6000);
      robotState = GRAB_PANEL_45; 
      break; 
    
    case GRAB_PANEL_45: 
      //straight (3); 
      closeGripper(); 
      delay(5000);
      //liftOff45(); 
      delay(2000);
      straight(-4); 
      delay(2000);
      turn(180); 
      delay(2000);
      lowerArm();
      delay(4000);
      straight (3); 
      robotState = APPROACH_BLOCK; 
      break; 
    
    case APPROACH_BLOCK: 
      turn(-90);
      delay(2000); 
      straight (7); 
      delay(2000);
      delay(500);
      robotState = IDLE; 
      break; 
    
    case PLACE_BLOCK: 
      lowerArmBlock(); 
      delay(2000);
      openGripper(); 
      delay(2000);
      straight (-3); 
      delay(2000);
      robotState = IDLE; 
      break; 

    case OPPOSITE_SIDE: 
      turn(90); 
      delay(2000);
      straight(32); 
      delay(2000);
      turn(90); 
      delay(2000);
      robotState = IDLE; 
      break; 

    case APPROACH_ROOF_25: 
      straight (2); 
      delay(2000);
      armRaise25(); 
      robotState = IDLE;
      break; 
    
    case GRAB_PANEL_25: 
      straight(6);
      delay(2000);
      closeGripper(); 
      delay(2000);
      liftOff25(); 
      delay(2000);
      straight(-5); 
      robotState = IDLE; 
      break; 
  }
}

void loop()
{
  //I THINK ALL OF THIS CAN BE DELETED 
  // LINE FOLLOW
  // if (leftLine.readVoltage() > 0.6 && rightLine.readVoltage() > 0.6) // stop at intersection
  // {

  //   left_motor.setSpeed(0);
  //   right_motor.setSpeed(0);
  //   Serial.printf("linetracker: left: %f, right %f\n", leftLine.readVoltage(), rightLine.readVoltage());
  // }
  // else
  // {
  //   lineFollow(leftV, rightV);
  // }

  // PRINT LINE SENSOR VALUES
  //printValues(leftV, rightV);

  // TURN
  //turn(90);

  // ULTRASONIC READ
  // if (ultrasonicRead() > 11)
  // {
  //   left_motor.setSpeed(60);
  //   right_motor.setSpeed(60);
  // }
  // else
  // {
  //   left_motor.setSpeed(0);
  //   right_motor.setSpeed(0);
  // }

  // STRAIGHT
   //straight(-1);

  // GRIPPER
  //  grip.write(openGrip);
  // delay(100);
  //  grip.write(closeGrip);
  // delay(100);

  // raise arm to a certain height
  //PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount); // have to test to find this number
  //setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);

  //FOR BRINGING ARM UP AND DOWN QUICKLY
  //bringArmDown();
  //bringArmUp();
  
  //checking the 25 roof 
  // openGripper(); 
  // armRaise25(); 
  //closeGripper(); 
  //liftOff25(); 
  //straight(-2); 
  //turn(180);
armRaise45(); 


  //ACTUAL PROGRAM
  // while (true)
  // {
  //   leftV = leftLine.readVoltage();
  //   rightV = rightLine.readVoltage();
  //   keyPress = decoder.getKeyCode();
  //   updateRobotState();
  // }

  // findEncoderCount();

  // if(firstTime) {
  //   PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount);
  //   setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
  //   firstTime = false;
  // }
  // comeBackDown();
}