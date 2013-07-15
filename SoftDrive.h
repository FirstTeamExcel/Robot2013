#include "Wpilib.h"

#ifndef SOFTDRIVE_H_
#define SOFTDRIVE_H_

#define FPGA_TIME_TO_MINUTES_FACTOR (60*1000*1000)

class SoftDrive : private RobotDrive
{
public:
    
    SoftDrive(SpeedController *_frontLeftMotor, SpeedController *_rearLeftMotor,
              SpeedController *_frontRightMotor, SpeedController *_rearRightMotor);
    
    void BeginDriveStraight(float _gyroAngle, float _maxOutput, float _gyroAngle, float _targetAngle);
    float ContinueDriveStraight(float _gyroAngle);
    void ResetDriveStraight();
    
    void SoftTankDrive(float leftValue, float rightValue);
        
    void SetIsStable(bool stable){isStable = stable;}
    bool IsStable(){return isStable;}
private:  //shooter attributes
    Timer gyro_off_target_timer;
    
    float forwardTippyAccelleration;
    float forwardStableAccelleration;
    float reverseTippyAccelleration;
    float reverseStableAccelleration;
    
    bool isStable;

    
    //Drive Straight stuff
    float maxOutput;
    float timeToDrive;
    Timer timeDriving;
    
    float targetAngle;
    
    float autonTurnAmount;
    float autonSpeedCorrect;
    float currentOutput;
    
    
    //Tank drive stuff
    float leftOutput;
    float rightOutput;
    UINT32 lastTime;
    
    float GetStep(float direction);
    float GetOutput(float targetOutput, float currentOutput);
    float GetElapsedSeconds();
};
#endif //SOFTDRIVE_H_
