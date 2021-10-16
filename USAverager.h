#pragma once

class USAverager {
    public:
        void setup();
        float getDistance();

    private:
        float distances [10];
        int count;
};