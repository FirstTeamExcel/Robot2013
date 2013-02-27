#include "Wpilib.h"

#ifndef COLLECTOR_H_
#define COLLECTOR_H_

class Collector
{
public:
    typedef enum {UP, DOWN, TRAVELING_DOWN, TRAVELING_UP, STARTING_POSITION, LEAVING_STARTING_POSITION}COLLECTOR_POSITION_TYPE;
private: //Keep out Connor!!!
    COLLECTOR_POSITION_TYPE position;
    float loadSpeed;
    float feedSpeed;
    float collectSpeed;
    Talon frontCollector;
    Talon backCollector;
    Solenoid lifterLeftExtend;
    Solenoid lifterLeftRetract;
    Solenoid lifterRightExtend;
    Solenoid lifterRightRetract;
    Solenoid tilterExtend;
    Solenoid tilterRetract;
    Timer timeTraveling;
    
    
public:
    Collector (int front_collector_channel,
            int back_collector_channel,
            int lifter_left_extend_channel,
            int lifter_left_retract_channel,
            int lifter_right_extend_channel,
            int lifter_right_retract_channel,
            int tilter_extend_channel,
            int tilter_retract_channel);
    float GetCollectSpeed(void){return collectSpeed;}
    void SetCollectSpeed(float speed){collectSpeed = speed;}
    float GetFeedSpeed(void) {return feedSpeed;}
    void SetFeedSpeed(float speed) {feedSpeed = speed;}
    float GetLoadSpeed(void) {return loadSpeed;}
    void SetLoadSpeed(float speed) {loadSpeed = speed;}
    COLLECTOR_POSITION_TYPE GetPosition (void){return position;}
    void Feed (void);
    void Load (void);
    void Collect (void);
    void Idle (void);
    void EnterStartingPosition (void);
    void LeaveStartingPosition (void);
};

#endif



