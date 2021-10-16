#pragma once

class Odometer{
    public:
    void setup(); //Do some math and stuff
    void iterateRight(); //Call upon right encoder tick
    void iterateLeft(); //Call upon left encoder tick
    double getX(); //Get X position of robot
    double getY(); //Get Y position of robot
    double getHeading(); //Get heading of robot CCW from Y axis
    private:
    double x = 0; //X position of robot
    double y = 0; //Y position of robot
    double heading = 0; //Heading of robot CCW from Y axis (Robot starts parallel to Y axis)
    const double HWT = 7.0/2.0; //Half of the wheeltrack in CM
    const double d = (HWT*2)/(360.0*4.0); //Distance that a wheel moves in one tick
    const double deltaTheta = (360.0*d)/(3.14159265*4.0*HWT); //Angle that the robot rotates in one tick in degrees
    double deltaThetaRAD;
    double sinDeltaTheta;
    double cosDeltaTheta;
};