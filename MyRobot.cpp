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
    SOLENOID_COLLECTOR_LIFTER_EXTEND,
    SOLENOID_COLLECTOR_LIFTER_RETRACT,
    SOLENOID_ROBOT_LIFTER_EXTEND,
    SOLENOID_ROBOT_LIFTER_RETRACT,
    SOLENOID_TILTER_EXTEND,
    SOLENOID_TILTER_RETRACT,
    SOLENOID_SHOOTER_EXTEND,
    SOLENOID_SHOOTER_RETRACT,
    SOLENOID_CLIMBER_EXTEND,
    SOLENOID_CLIMBER_RETRACT
}SOLENOID_CHANNEL;
typedef enum
{
    DIGITAL_0,
    DIGITAL_COMPRESSOR_SWITCH,
    DIGITAL_SHOOTER_SENSOR,
    DIGITAL_WHEEL_RIGHT_FORWARD,
    DIGITAL_WHEEL_RIGHT_BACKWARD,
    DIGITAL_WHEEL_LEFT_FORWARD,
    DIGITAL_WHEEL_LEFT_BACKWARD,
    DIGITAL_LEFT_COLLECTOR_SWITCH,
    DIGITAL_RIGHT_COLLECTOR_SWITCH,
    DIGITAL_HOPPER_SWITCH
        
}DIGITAL_IO_CHANNEL;
typedef enum
{
    RELAY_0,
    RELAY_COMPRESSOR
}RELAY_CHANNEL;
typedef enum 
	{
		AUTONOMOUS_DROP_COLLECTOR,
		AUTONOMOUS_GET_IN_POSITION,
		AUTONOMOUS_FIRING_SHOT,
		AUTONOMOUS_SHOT_FIRED,
		AUTONOMOUS_REARMING_SHOT,
		AUTONOMOUS_COLLECT_FRISBEE,
		AUTONOMOUS_RELOADING,
		AUTONOMOUS_DONE,
		
	}AUTONOMOUS_STATE;
	
	AUTONOMOUS_STATE autonomousState;

typedef enum
{
	AUTONOMOUS_MODE_TWO_FRISBEE,
	AUTONOMOUS_MODE_FOUR_FRISBEE,
	AUTONOMOUS_MODE_THREE_FRISBEE,
	AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD,
	AUTONOMOUS_MODE_FIVE_FRISBEE_BACK,
	AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD,
	AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK,
	AUTONOMOUS_MODE_FEED_FRISBEE,
	AUTONOMOUS_MODE_SIT_AND_SHOOT
}AUTONOMOUS_MODE_SELECT;

AUTONOMOUS_MODE_SELECT autonomousMode;


class RobotDemo : public IterativeRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick leftStick; // right joystick
	Joystick rightStick;// left joystick
	Joystick operatorStick;
	Timer timeInAutonomous;
	//DigitalInput shooterSensor;
	Counter shooterSensor;
	Shooter frisbeeShooter;
	Solenoid robotLifterExtend;
	Solenoid robotLifterRetract;
	//TargetCamera targetCamera;

	DriverStationLCD *driverStationLCD;
//	Solenoid lightSensorPower;
	Collector collector;
	Compressor compressor;
	//Climber climber;
	DriverStation *driverStation;
	DigitalInput limitSwitchLeft;
	DigitalInput limitSwitchRight;
	DigitalInput limitSwitchLoader;
	Timer autonDrivingBack;
	Timer autonDrivingForward;
	Timer autonLoading;
	Timer autonShooting;
	int autonShotCount;
	int autonStepCount;
	bool autonReset;	

public:
	RobotDemo(void): //initialized
		myRobot(PWM_LEFT_DRIVE,PWM_RIGHT_DRIVE),	// these must be initialized in the same order
		leftStick(1), // as they are declared above.
		rightStick(2),
		operatorStick(3),
		timeInAutonomous(),
		shooterSensor (DIGITAL_SHOOTER_SENSOR),
		frisbeeShooter(PWM_SHOOTER_WHEEL,&shooterSensor, SOLENOID_SHOOTER_EXTEND, SOLENOID_SHOOTER_RETRACT),
		robotLifterExtend(SOLENOID_ROBOT_LIFTER_EXTEND),
		robotLifterRetract(SOLENOID_ROBOT_LIFTER_RETRACT),
		//targetCamera (),
//		lightSensorPower(2),
		collector(PWM_FRONT_COLLECTOR,
		        PWM_BACK_COLLECTOR,
		        SOLENOID_COLLECTOR_LIFTER_EXTEND,
		        SOLENOID_COLLECTOR_LIFTER_RETRACT,
		        SOLENOID_TILTER_EXTEND,
		        SOLENOID_TILTER_RETRACT),
//		climber(SOLENOID_CLIMBER_EXTEND,
//				SOLENOID_CLIMBER_RETRACT)
        compressor (DIGITAL_COMPRESSOR_SWITCH,RELAY_COMPRESSOR),
        limitSwitchLeft(DIGITAL_LEFT_COLLECTOR_SWITCH),
        limitSwitchRight(DIGITAL_RIGHT_COLLECTOR_SWITCH),
        limitSwitchLoader(DIGITAL_HOPPER_SWITCH),
        autonDrivingBack(),
        autonDrivingForward(),
        autonLoading(),
        autonShooting()
           
		
		
	{
		// Acquire the Driver Station object
		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();
		myRobot.SetExpiration(0.1);
		frisbeeShooter.SetCountsPerRevolution(9);
		autonReset = true;
	}

	void DisabledInit(void)
	{
		
	}
	
	void TeleopInit (void)
	{
	    compressor.Start();
	    collector.LeaveStartingPosition();
	    //lightSensorPower.Set(true);
	}
	
	void AutonomousInit (void)
	{
	    collector.EnterStartingPosition();
	    compressor.Start();
		timeInAutonomous.Reset();
		timeInAutonomous.Start();
		autonDrivingBack.Stop();
		autonDrivingForward.Stop();
		autonLoading.Stop();
		autonShooting.Stop();
		autonDrivingBack.Reset();
		autonDrivingForward.Reset();
		autonLoading.Reset();
		autonShooting.Reset();
		autonStepCount = 0;
		autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK;
		autonShotCount = 0;
		autonReset = true;
	}
	
	void DisabledPeriodic(void)
	{
		static Timer button_combo_timer;
		
		if (rightStick.GetRawButton(2) == true,
			leftStick.GetRawButton(2) == true)
		{
			button_combo_timer.Start();
		}
		else
		{
			button_combo_timer.Reset();
			button_combo_timer.Stop();
		}
		
		if (button_combo_timer.Get() > 2.00)
		{
			button_combo_timer.Reset();
			switch (autonomousMode)
			{
			case AUTONOMOUS_MODE_FIVE_FRISBEE_BACK:
				autonomousMode = AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD;
				break;
			case AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD:
				autonomousMode = AUTONOMOUS_MODE_THREE_FRISBEE;
				break;
			case AUTONOMOUS_MODE_THREE_FRISBEE:
				autonomousMode = AUTONOMOUS_MODE_TWO_FRISBEE;
				break;
			case AUTONOMOUS_MODE_TWO_FRISBEE:
				autonomousMode = AUTONOMOUS_MODE_FOUR_FRISBEE;
				break;
			case AUTONOMOUS_MODE_FOUR_FRISBEE:
				autonomousMode = AUTONOMOUS_MODE_SIT_AND_SHOOT;
				break;
			case AUTONOMOUS_MODE_SIT_AND_SHOOT:
				autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD;
				break;
			case AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD:
				autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK;
				break;
			case AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK:
				autonomousMode = AUTONOMOUS_MODE_FEED_FRISBEE;
				break;
			default:
			case AUTONOMOUS_MODE_FEED_FRISBEE:
				autonomousMode = AUTONOMOUS_MODE_FIVE_FRISBEE_BACK;
				break;
			}
		}	

		switch (autonomousMode)
		{
		default:
		case AUTONOMOUS_MODE_FIVE_FRISBEE_BACK:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 3, pick up center frisbees, shoot 2 more");
				break;
		case AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 3, pick up pyramid frisbees, shoot 2 more");
				break;
		case AUTONOMOUS_MODE_THREE_FRISBEE:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 3 frisbees. That's it.");
				break;
		case AUTONOMOUS_MODE_TWO_FRISBEE:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 2 from in front of the pyramid somehow");
				break;
		case AUTONOMOUS_MODE_FOUR_FRISBEE:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 2 from in front of the pyramid, then drive back and pick up and shoot 2 more");
				break;
		case AUTONOMOUS_MODE_SIT_AND_SHOOT:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 3, and then get fed more by another robot");
				break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 3, pick up pyramid and then pyramid frisbees and shoot 4 more.  Do a victory dance.");
				break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 3, pick up center and then pyramid frisbees and shoot 4 more.  Do a victory dance.");
				break;
		case AUTONOMOUS_MODE_FEED_FRISBEE:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Feed frisbees to another robot");
				break;
		}

		driverStationLCD->UpdateLCD();
	}

	
	void TeleopPeriodic (void)
	{

	    compressor.Start();
		//targetCamera.SetDebugMode(operatorStick.GetRawButton(9));
			
		if ((operatorStick.GetRawButton(1)==true)||(leftStick.GetRawButton(1)==true))
		{
		    //frisbeeShooter.SetRpm(1000);
		    //frisbeeShooter.SetRpm(((1-operatorStick.GetThrottle())/2)*10000);
			frisbeeShooter.SetPower((1-operatorStick.GetThrottle())/2);
		}
		else 
		{
		    //frisbeeShooter.SetRpm(0);
			frisbeeShooter.SetPower (0);
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
			//float load_speed = ((1-operatorStick.GetThrottle())/2);
			
			//collector.SetLoadSpeed(load_speed);
			//driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "load speed: %f",load_speed);
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
		
		if (((operatorStick.GetRawButton(11)==true )||(rightStick.GetRawButton(1))) &&
				((frisbeeShooter.GetPreviousRpm() != 0) || (frisbeeShooter.GetPower()!=0)))
		{
			//shoot frisbees
			frisbeeShooter.ShootFrisbee(true);
		}
		else
		{
			frisbeeShooter.ShootFrisbee(false);
		}
		if (leftStick.GetRawButton(3)  && rightStick.GetRawButton(3))
		{
			robotLifterExtend.Set(true);
			robotLifterRetract.Set(false);
		}
		else if (leftStick.GetRawButton(2)  && rightStick.GetRawButton(2))
		{
			robotLifterRetract.Set(true);
			robotLifterExtend.Set(false);
		}
		//frisbeeShooter.ControlSpeed();
		myRobot.TankDrive(-leftStick.GetY(), -rightStick.GetY()); // drive with arcade style (use right stick)
		//float distance = targetCamera.GetDistance();
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "rpm: %f", frisbeeShooter.GetPreviousRpm());
        //driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "distance: %f",distance);
		driverStationLCD->UpdateLCD();			
	}
		
	void AutonomousPeriodic (void)
	{
		switch (autonomousMode)
		{
		case AUTONOMOUS_MODE_FIVE_FRISBEE_BACK:
			break;
		case AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD:
			//Third
			break;
		case AUTONOMOUS_MODE_THREE_FRISBEE:
			//Second
			break;
		case AUTONOMOUS_MODE_TWO_FRISBEE:
			break;
		case AUTONOMOUS_MODE_FOUR_FRISBEE:
			break;
		case AUTONOMOUS_MODE_SIT_AND_SHOOT:
			break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD:
			break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK:
			//First
			AutonSevenFrisbeeBack();
			break;
		default:
		case AUTONOMOUS_MODE_FEED_FRISBEE:
			break;
		}

		driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "Auton Step: %d", autonStepCount);
		driverStationLCD->UpdateLCD();
	}
	
	bool AutonomousShoot(int quantity, bool use_sensor, bool reset = false)
	{
		if (reset == true)
		{
			autonShotCount = 0;
		}
		frisbeeShooter.ShootFrisbee(false);
		frisbeeShooter.SetPower(1.0);
		
		if(autonShotCount <= quantity)
		{
			if (1/*(use_sensor != true) || limitSwitchLoader.Get() == true*/)
			{
				if (frisbeeShooter.ShootFrisbee(true) == true)
				{
					autonShotCount ++;
				}
			}
			return false;
		}
		else
		{
			return true;
		}
		return false;
	}
	bool AutonomousCollectBack(float distance, float time, bool collector_enabled = true, bool reset = false)
	{
		if (reset == true)
		{
			//TODO reset encoders
			autonDrivingBack.Reset();
			autonDrivingBack.Start();
		}
		
		if (autonDrivingBack.Get() > time /*|| (limitSwitchLeft.Get() && limitSwitchRight.Get()) || distance_traveled > distance*/)
		{
			myRobot.Drive(0.0, 0.0);
			return true;//Stop motors and return true
		}
		else if (autonDrivingBack.Get() > 0.5)
		{
			myRobot.Drive(-0.75, 0.0);//Set speed to 50% and collect
			collector.Collect();
			return false;//Return false
		}
		else
		{
			myRobot.Drive(-0.5, 0.0);//Set speed to 50% and collect
			collector.Collect();
			return false;//Return false
		}
		
		return false;
	}
	bool AutonomousCollectForward(float distance, float time, bool collector_enabled = true, bool reset = false)
	{
		if (reset == true)
		{
			//TODO reset encoders
			autonDrivingForward.Reset();
			autonDrivingForward.Start();
		}

		if (autonDrivingForward.Get()> time /*|| (limitSwitchLeft.Get() && limitSwitchRight.Get()) || distance_traveled > distance*/)
		{
			myRobot.Drive(0.0, 0.0);
			return true;//Stop motors and return true
		}
		else if (autonDrivingBack.Get() > 0.5)
		{
			myRobot.Drive(0.75, 0.0);//Set speed to 50% and collect
			collector.Collect();
			return false;//Return false
		}
		else
		{
			myRobot.Drive(0.5,0.0);//Set speed to 50% and collect
			collector.Collect();
			return false;//Return false
		}
		
		return false;
	}
	
	bool AutonomousLoadFrisbees(bool use_switch = true, bool reset = false)
	{
		if (reset == true)
		{
			autonLoading.Reset();
			autonLoading.Start();
		}
		
		if ((autonLoading.Get() > 3.0) /*|| (limitSwitchLoader.Get() && use_switch)*/)
		{
			collector.Idle();
			return true;
//				 doughnuts = true;
//				 pinball = true;
//				 worms = true;		 
		}
		else
		{
			collector.Load();
			return false;
		}
		return false;	
	}
	
	void AutonSevenFrisbeeBack(void)
	{
		bool done_loading = false;
		frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
		switch (autonStepCount)
		{

		case 0:
			//Drive forward slowly to detatch from bar
			if (autonReset == true)
			{
				autonDrivingForward.Reset();
				autonDrivingForward.Start();
			}
			frisbeeShooter.SetPower(0.9);
			collector.LeaveStartingPosition();
			if (autonDrivingForward.Get() > 0.2)
			{
				myRobot.Drive(0.0,0.0);
				autonReset = true;
				autonStepCount++;
				collector.Collect();
			}
			else
			{
				myRobot.Drive(0.30,0.0);
				autonReset = false;
			}
			break;
		case 1:
			if (collector.GetPosition() == Collector::UP)
				collector.Collect();
			if (AutonomousShoot(3,true,autonReset))
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			//Shoot 3 frisbees (3sec)
			break;
		case 2:
			if (AutonomousCollectForward(999.0,0.40,true,autonReset)== true)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			//Drive forward 2.5sec
			break;
		case 3:
			myRobot.Drive(0.0,0.0);
			done_loading = AutonomousLoadFrisbees(true,autonReset);
			if (autonLoading.Get() > 0.5)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 4:
			//Load (for 3 sec) and
			//Drive backward 4.5 sec
			done_loading = AutonomousLoadFrisbees(true,false);
			if (AutonomousCollectBack(999.0,2.00,done_loading,autonReset) == true)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 5:
			myRobot.Drive(0.0,0.0);
			done_loading = AutonomousLoadFrisbees(true,autonReset);
			if (autonLoading.Get() > 0.5)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 6:
			//Load and
			//Drive Forward 2.5 sec
			done_loading = AutonomousLoadFrisbees(true,false);
			if (AutonomousCollectForward(999.0,1.3,done_loading,autonReset) == true)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 7:
			//Shoot 4 frisbees (4 seconds)
			if (AutonomousShoot(4,true,autonReset))
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 8:
			autonReset = true;
			break;
		case 9:
			break;
		case 10:
			break;
		}
	}

	void AutonThreeFrisbee(void)
	{
		
	}
	void AutonFiveFrisbeeForward(void)
	{
	}
};

START_ROBOT_CLASS(RobotDemo);

