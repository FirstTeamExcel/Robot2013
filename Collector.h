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
    Solenoid lifterExtend;
    Solenoid lifterRetract;
    Solenoid tilterExtend;
    Solenoid tilterRetract;
    Solenoid extraLifterExtend;
    Solenoid extraLifterRetract;
    ADXL345_I2C accelerometer;
    DriverStation *ds;
    Timer timeTraveling;
    
    void TravelingUp(void);
public:
    Collector (int front_collector_channel,
            int back_collector_channel,
            int lifter_extend_channel,
            int lifter_retract_channel,
            int tilter_extend_channel,
            int tilter_retract_channel);
    float GetCollectSpeed(void){return collectSpeed;}
    void SetCollectSpeed(float speed){collectSpeed = speed;}
    float GetFeedSpeed(void) {return feedSpeed;}
    void SetFeedSpeed(float speed) {feedSpeed = speed;}
    float GetLoadSpeed(void) {return loadSpeed;}
    void SetLoadSpeed(float speed) {loadSpeed = speed;}
    COLLECTOR_POSITION_TYPE GetPosition (void){return position;}
    void Feed (bool feed_forward = false);
    void Load (float temp_load_speed = 0.0);
    void Collect (void);
    void Idle (void);
    bool Raise(void);
    bool Lower(void);
    void EnterStartingPosition (void);
    void LeaveStartingPosition (void);
};

#endif



