#include "Odometry.h"
#include <RBE1001Lib.h>

extern LeftMotor left;
extern RightMotor right;

void Odometer::setup(){
    deltaThetaRAD = deltaTheta*DEG_TO_RAD;
    cosDeltaTheta = cos(deltaThetaRAD);
    sinDeltaTheta = sin(deltaThetaRAD);
}

void Odometer::iterateRight(){
    double headingComplement = 90-heading; //Find 90-heading
    double driveCenterX = x-HWT*sin((headingComplement)*DEG_TO_RAD); //Find the drive center x pos
    double driveCenterY = y-HWT*cos((headingComplement)*DEG_TO_RAD); //Find the drive center y pos
    double xOrigin = x-driveCenterX; //Find x if the drive center is the origin
    double yOrigin = y-driveCenterY; //Find y if the drive center is the origin
    x = xOrigin + xOrigin*cosDeltaTheta-yOrigin*sinDeltaTheta; //Iterate x value with rotation matrix
    y = yOrigin + xOrigin*sinDeltaTheta-yOrigin*cosDeltaTheta; //Iterate y value with rotation matrix
    heading += deltaTheta; //Iterate heading
}

void Odometer::iterateLeft(){

}