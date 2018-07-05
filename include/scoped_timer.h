#ifndef KINECT_SCOPED_TIMER_H
#define KINECT_SCOPED_TIMER_H

#include<ctime>

class ScopedTimer
{
public:

    ScopedTimer(const char* name, bool only_for_logging = true, bool unconditional_logging=false);
    ~ScopedTimer();
    double elapsed();

private:
    struct timespec start;
    const char* name;
    bool triggering;
};


#endif
