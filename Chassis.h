#pragma once

class Chassis {
    public:
        void setup();
        void setConstants(float p, float i, float d);
        void follow();
        void followBackwards();
        void turn(bool leftTurn);
        void driveStraight();
        void stop();
    private:
        float Kp = 20;
        float Ki = 0;
        float Kd = 0;
        float integral;
        float lastError;
        unsigned long lastTime;
};