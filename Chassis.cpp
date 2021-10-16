#include "Chassis.h"
#include <RBE1001Lib.h>

extern LeftMotor left;
extern RightMotor right;
extern ESP32AnalogRead leftRead;
extern ESP32AnalogRead rightRead;

int baseSpeed = 100;

//Call to unwind, set up variables
void Chassis::setup(){
    integral = 0; //unwind
    lastError = 0; //reset last error
    lastTime = millis(); //set lastTime ran
}

//Set PID constants
void Chassis::setConstants(float p, float i, float d){
    Kp = p;
    Ki = i;
    Kd = d;
}

//Follow a line
void Chassis::follow(){
    float error = leftRead.readVoltage() - rightRead.readVoltage(); // proportion between sensors
    integral += error;                                           //integral
    float diff = error - lastError;                        //derivative

    float tare = (error * Kp) + (integral * Ki) + (diff * Kd); //how fast do we need to turn?

    left.setSpeed(baseSpeed - tare); //set motor speeds with turn
    right.setSpeed(baseSpeed + tare);
    lastError = error;
}

void Chassis::followBackwards(){
    float error = leftRead.readVoltage() - rightRead.readVoltage(); // proportion between sensors
    integral += error;                                           //integral
    float diff = error - lastError;                        //derivative

    float tare = (error * Kp) + (integral * Ki) + (diff * Kd); //how fast do we need to turn?

    left.setSpeed(-1 * baseSpeed - tare); //set motor speeds with turn
    right.setSpeed(-1 * baseSpeed + tare);
    lastError = error;
}

void Chassis::turn(bool leftTurn){
    if(leftTurn){
        left.setSpeed(-100);
        right.setSpeed(100);
    }
    else{
        left.setSpeed(100);
        right.setSpeed(-100);
    }
}

void Chassis::driveStraight(){
    left.setSpeed(100);
    right.setSpeed(100);
}

void Chassis::stop(){
    left.setSpeed(0);
    right.setSpeed(0);
}