#include "SoftDrive.h"
#define AUTON_STRAIGHTEN() myRobot.TankDrive((autonTurnAmount * -4.0), (autonTurnAmount * 4.0))
#define AUTON_SPEED_CORRECT_FACTOR 0.5
#define FPGA_TIME_TO_SECONDS_APPROX(usec) (usec >> 20)

SoftDrive::SoftDrive(SpeedController *_frontLeftMotor, SpeedController *_rearLeftMotor,
                SpeedController *_frontRightMotor, SpeedController *_rearRightMotor):
                RobotDrive(_frontLeftMotor, _rearLeftMotor, _frontRightMotor, _rearRightMotor),
                gyro_off_target_timer()
{
    currentOutput = 0.0;
    timeToDrive = 0.0;
    
}

void SoftDrive::BeginDriveStraight(float _gyroAngle, float _maxOutput, float _time, float _targetAngle = 0.0)
{
    if (timeDriving.Get() > timeToDrive + 1.0)
    {
        currentOutput = 0.0;
    }
    targetAngle = _targetAngle;
    maxOutput = _maxOutput;
    timeToDrive = _time;
    timeDriving.Reset();
    timeDriving.Start();
    ContinueDriveStraight(_gyroAngle);
}

float SoftDrive::ContinueDriveStraight(float _gyroAngle)
{

    float rotation = _gyroAngle - targetAngle;
    float direction = (maxOutput >= 0.0) ? 1.0 : -1.0;
    
    if ((rotation > 90.0) || (rotation < -90.0) || (gyro_off_target_timer.Get() > 1.0))
    {
        Drive(0.0,0.0);    
        return false; 
    }
    else if ((rotation > 30.0) || (rotation < -30.0))
    {
        gyro_off_target_timer.Start();
    }
    else
    {
        gyro_off_target_timer.Reset();
    }
    
    autonTurnAmount = rotation / 100.0f;
    if (autonTurnAmount > 0.1) autonTurnAmount = 0.1;
    if (autonTurnAmount < -0.1) autonTurnAmount = -0.1;
    
    autonSpeedCorrect = (autonTurnAmount) * AUTON_SPEED_CORRECT_FACTOR;
    if (autonSpeedCorrect < 0.0) autonSpeedCorrect = autonSpeedCorrect * -1.0;

    float step;
    if (direction > 0)
    {
        step = isStable ? forwardStableAccelleration : forwardTippyAccelleration;
    }
    else
    {
        step = isStable ? reverseStableAccelleration : reverseStableAccelleration;
    }
    
    //if the output is less than it should be
    if (currentOutput < maxOutput)
    {
        currentOutput += step;
        //If we are slowing down, double step
        if (direction < 0)
        {
            currentOutput += step;
        }
        if (currentOutput > maxOutput)
        {
            currentOutput = maxOutput;
        }
    }
    //If the ouptut is more than it should be
    else if (currentOutput > maxOutput)
    {
        currentOutput -= step;
        //If we are slowing down, double step
        if (direction > 0)
        {
            currentOutput -= step;
        }
        
        if (currentOutput < maxOutput)
        {
            currentOutput = maxOutput;
        }
    }
    
    float remainingTime = timeToDrive - timeDriving.Get();
    if (remainingTime > 0.0)
    {
        Drive(currentOutput, autonTurnAmount);
    }
    else
    {
        Drive(0.0,0.0);
        return 0.0;
    }
    
    return remainingTime;
}

void SoftDrive::ResetDriveStraight()
{
    currentOutput = 0.0;
    targetAngle = 0.0;
    maxOutput = 0.0;
    timeToDrive = 0.0;
}

void SoftDrive::SoftTankDrive(float leftValue, float rightValue)
{
    UINT32 _currentTime = FPGA_TIME_TO_SECONDS_APPROX(GetFPGATime());
    //long _elapsedTime = - lastTime;
    //lastTime = _currentTime;
    //both wheels in the same direction
    if (((leftValue < 0.0) && (rightValue < 0.0)) ||
        ((leftValue > 0.0) && (rightValue > 0.0)))   
    {
        
    }
}



