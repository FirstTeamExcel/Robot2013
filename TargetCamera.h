#include "Wpilib.h"

#ifndef TARGETCAMERA_H_
#define TARGETCAMERA_H_

class TargetCamera
{
private:
    float lastDistance;
    unsigned long targetTime;
    unsigned long refreshRate;
    int ComputeDistance (void);
    bool debugMode;
    
public:
    TargetCamera(void);
    float GetLastDistance(void);
    float GetDistance(void);
    bool GetDebugMode(void);
    void SetDebugMode(bool debug);
    unsigned long GetRefreshRate(void);
    void SetRefreshRate(unsigned long rate);
	
};

#endif
