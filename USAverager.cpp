#include <RBE1001Lib.h>
#include "USAverager.h"


extern Rangefinder us;

void USAverager::setup()
{
    for(int i=0; i<10; i++){
        distances[i] = 0;
    }
    count = 0;
}

float USAverager::getDistance(){
    distances[count % 10] = us.getDistanceCM();\
    
    count++;

    float sum = 0;

    for(float i:distances){
        sum += i;
    }
    return sum / 10;
}