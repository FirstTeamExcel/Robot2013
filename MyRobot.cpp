#include "WPILib.h"
#include "Shooter.h"
#include "TargetCamera.h"
#include "Collector.h"
#include "Donuts"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
  
 * lower_case = function parameters, local variables
 * camelCase = class attributes, functions
 * Upper_CASE = constants, #define, enums
 * CapitilizedNames = classes, enum types 
 */

typedef enum
{
    PWM_0,
    PWM_RIGHT_DRIVE,
    PWM_LEFT_DRIVE,
    PWM_SHOOTER_WHEEL,
    PWM_BACK_COLLECTOR,
    PWM_FRONT_COLLECTOR,
    
}PWM_CHANNEL;
typedef enum
{
    SOLENOID_0,
    SOLENOID_LIFTER_RIGHT_EXTEND,
    SOLENOID_LIFTER_RIGHT_RETRACT,
    SOLENOID_LIFTER_LEFT_EXTEND,
    SOLENOID_LIFTER_LEFT_RETRACT,
    SOLENOID_TILTER_EXTEND,
    SOLENOID_TILTER_RETRACT,
    SOLENOID_SHOOTER_EXTEND,
    SOLENOID_SHOOTER_RETRACT,
}SOLENOID_CHANNEL;
typedef enum
{
    DIGITAL_0,
    DIGITAL_COMPRESSOR_SWITCH,
    DIGITAL_SHOOTER_SENSOR
}DIGITAL_IO_CHANNEL;
typedef enum
{
    RELAY_0,
    RELAY_COMPRESSOR
}RELAY_CHANNEL;


class RobotDemo : public IterativeRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick leftStick; // right joystick
	Joystick rightStick;// left joystick
	Joystick operatorStick;
	Timer TimeInAutonomous;
	//DigitalInput shooterSensor;
	Counter shooterSensor;
	Shooter frisbeeShooter;
	TargetCamera targetCamera;
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;
//	Solenoid lightSensorPower;
	Collector collector;
	Compressor compressor;
	
	

public:
	RobotDemo(void): //initialized

		//myRobot.Drive(forwardSpeed, curve); curve less than 0 turns left, curve greater than 0 turns right
		myRobot(PWM_LEFT_DRIVE,PWM_RIGHT_DRIVE),	// these must be initialized in the same order
		
		leftStick(1), // as they are declared above.
		rightStick(2),
		operatorStick(3),
		TimeInAutonomous(),
		shooterSensor (DIGITAL_SHOOTER_SENSOR),
		frisbeeShooter(PWM_SHOOTER_WHEEL,&shooterSensor),
		targetCamera (),
//		lightSensorPower(2),
		collector(PWM_FRONT_COLLECTOR,
		        PWM_BACK_COLLECTOR,
		        SOLENOID_LIFTER_LEFT_EXTEND,
		        SOLENOID_LIFTER_LEFT_RETRACT,
		        SOLENOID_LIFTER_RIGHT_EXTEND,
		        SOLENOID_LIFTER_RIGHT_RETRACT,
		        SOLENOID_TILTER_EXTEND,
		        SOLENOID_TILTER_RETRACT),
        compressor (DIGITAL_COMPRESSOR_SWITCH,RELAY_COMPRESSOR)
		        
		
		
	{
		// Acquire the Driver Station object
		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();
		myRobot.SetExpiration(0.1);
		frisbeeShooter.SetCountsPerRevolution(9);
	}

	void DisabledInit(void)
	{
		
	}
	
	void TeleopInit (void)
	{
	    compressor.Start();
	    //lightSensorPower.Set(true);
	}
	
	void AutonomousInit (void)
	{
	    collector.EnterStartingPosition();
	    compressor.Start();
		TimeInAutonomous.Reset();
		TimeInAutonomous.Start();
	}
	
	void DisabledPeriodic(void)
	{
			
	}
		
	void TeleopPeriodic (void)
	{
		targetCamera.SetDebugMode(operatorStick.GetRawButton(9));
			
		if (operatorStick.GetRawButton(1)==true )
		{
		    //frisbeeShooter.SetRpm(1000);
		    frisbeeShooter.SetRpm(((1+operatorStick.GetThrottle())/2)*10000);
			//frisbeeShooter.SetPower((1+operatorStick.GetThrottle())/2);
		}
		else 
		{
		    frisbeeShooter.SetRpm(0);
			//frisbeeShooter.SetPower (0);
		}
		
		//Collector
		if (operatorStick.GetRawButton(2)==true )
		{   
			//pick up from floor
			collector.Collect();
		}
		else if (operatorStick.GetRawButton(3)==true )
		{
			//load shooter
			collector.Load();
		}		
		else if (operatorStick.GetRawButton(7)==true )
		{
			//barf frisbees
			collector.Feed();
		}
		else 
		{
			//stop collector
			collector.Idle();
		}
		frisbeeShooter.ControlSpeed();
		myRobot.TankDrive(-leftStick.GetY(), -rightStick.GetY()); // drive with arcade style (use right stick)
		float distance = targetCamera.GetDistance();
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "rpm: %f", frisbeeShooter.GetPreviousRpm());
        driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "distance: %f",distance);
        driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "switch: %d",shooterSensor.Get());
		driverStationLCD->UpdateLCD();			
	}
		
	void AutonomousPeriodic (void)
	{
		if (TimeInAutonomous.Get() < 5.0)
		{
			myRobot.Drive(.75 - (TimeInAutonomous.Get()*.1), 0.0); 	// drive forwards full speed	
	    }
		else
		{
		    myRobot.Drive(0.0, 0.0); 	// stop robot
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

