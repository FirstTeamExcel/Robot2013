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
#define TIME_AUTONOMOUS_DISLODGE 0.7
#define TIME_AUTONOMOUS_SPIN_UP 0.5
#define POWER_AUTONOMOUS_SHOTS 0.56

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
    DIGITAL_WHEEL_RIGHT_FORWARD,
    DIGITAL_WHEEL_RIGHT_BACKWARD,
    DIGITAL_WHEEL_LEFT_FORWARD,
    DIGITAL_WHEEL_LEFT_BACKWARD,
    DIGITAL_LEFT_COLLECTOR_SWITCH,
    DIGITAL_RIGHT_COLLECTOR_SWITCH,
    DIGITAL_HOPPER_SWITCH,
    DIGITAL_LEFT_BRAKE_OUT,
    DIGITAL_RIGHT_BRAKE_OUT
        
}DIGITAL_IO_CHANNEL;
typedef enum
{
    RELAY_0,
    RELAY_1,
    RELAY_COMPRESSOR
}RELAY_CHANNEL;
typedef enum
{
	ANALOG_0,
	ANALOG_GYRO
}ANALOG_CHANNEL;
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
	Gyro gyro;
	DigitalOutput rightBrake;
	DigitalOutput leftBrake;
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
	Encoder rightEncoder;
	Encoder leftEncoder;
	int autonShotCount;
	int autonStepCount;
	float autonTurnAmount;
	bool autonReset;	

public:
	RobotDemo(void): //initialized
		myRobot(PWM_LEFT_DRIVE,PWM_RIGHT_DRIVE),	// these must be initialized in the same order
		leftStick(1), // as they are declared above.
		rightStick(2),
		operatorStick(3),
		timeInAutonomous(),
		//shooterSensor (DIGITAL_SHOOTER_SENSOR),
		frisbeeShooter(PWM_SHOOTER_WHEEL,&shooterSensor, SOLENOID_SHOOTER_EXTEND, SOLENOID_SHOOTER_RETRACT),
		robotLifterExtend(SOLENOID_ROBOT_LIFTER_EXTEND),
		robotLifterRetract(SOLENOID_ROBOT_LIFTER_RETRACT),
		gyro(ANALOG_GYRO),
		rightBrake(DIGITAL_RIGHT_BRAKE_OUT),
		leftBrake(DIGITAL_LEFT_BRAKE_OUT),
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
        autonShooting(),
        rightEncoder(DIGITAL_WHEEL_RIGHT_FORWARD, DIGITAL_WHEEL_RIGHT_BACKWARD),
        leftEncoder(DIGITAL_WHEEL_LEFT_FORWARD, DIGITAL_WHEEL_LEFT_BACKWARD)
		
		
	{
		autonomousMode = AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD;
		// Acquire the Driver Station object
		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();
		myRobot.SetExpiration(0.1);
		
		autonReset = true;
	}

	void DisabledInit(void)
	{
		leftBrake.Set(true);
		rightBrake.Set(true);
		autonomousMode = AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD;
		compressor.Stop();
	}
	
	void TeleopInit (void)
	{
		myRobot.SetSafetyEnabled(true);
	    compressor.Start();
	    leftBrake.Set(true);
	    rightBrake.Set(true);
	    
	    collector.LeaveStartingPosition();
	    //lightSensorPower.Set(true);
	}
	
	void AutonomousInit (void)
	{
		myRobot.SetSafetyEnabled(false);
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
		autonShotCount = 0;
		autonReset = true;
		leftEncoder.Start();
		rightEncoder.Start();
		autonTurnAmount = 0;
		gyro.Reset();
		leftBrake.Set(false);
		rightBrake.Set(false);
	}

	void TeleopPeriodic (void)
	{
		static bool stinger_toggled = false;
	    compressor.Start();
		//targetCamera.SetDebugMode(operatorStick.GetRawButton(9));
	    float shot_power = 0;
	    float rpm = frisbeeShooter.GetRpm();
	//Test code
	    static float test_rpm = 0;

		if (operatorStick.GetRawButton(6) == true)
		{
			test_rpm = rpm;	
		}
		if (operatorStick.GetRawButton(7) == true)
		{
			frisbeeShooter.SetRpm(int(test_rpm));
		}
		else
		{
	//end Test code
			if ((operatorStick.GetRawButton(1)==true)||(leftStick.GetRawButton(1)==true))
			{
				//frisbeeShooter.SetRpm(1000);
				//frisbeeShooter.SetRpm(((1-operatorStick.GetThrottle())/2)*10000);
				if (operatorStick.GetRawButton(11)==true)
				{
					shot_power = ((1-operatorStick.GetThrottle())/2);
				}
				else
				{
					shot_power = 0.65;
				}
			}
			frisbeeShooter.SetPower(shot_power);
		}
	    driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "Power: %f", shot_power);	
	    driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Rpm: %f", rpm);
	    driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "TRpm: %f", test_rpm);			
		
		//Collector
		if ((operatorStick.GetRawButton(2)==true )||(leftStick.GetRawButton(2)==true))
		{   
			//pick up from floor
			collector.Collect();
		}
		else if ((operatorStick.GetRawButton(4)==true )||(rightStick.GetRawButton(2)==true))
		{
			collector.Load();
		}		
		else if (operatorStick.GetRawButton(3)==true )
		{
			//barf frisbees
			collector.Feed();
		}
		else 
		{
			//stop collector
			collector.Idle();
		}
		
		if ((operatorStick.GetRawButton(11)==true )||(rightStick.GetRawButton(1) == true))
		{
			//shoot frisbees
			frisbeeShooter.ShootFrisbee(true);
		}
		else
		{
			frisbeeShooter.ShootFrisbee(false);
		}

		if (leftStick.GetRawButton(3)  && rightStick.GetRawButton(3) && 
			(stinger_toggled == false) && (robotLifterExtend.Get() == false))
		{
			robotLifterExtend.Set(true);
			robotLifterRetract.Set(false);
			stinger_toggled = true;
		}
		else if (leftStick.GetRawButton(3)  && rightStick.GetRawButton(3) && 
				(stinger_toggled == false))
		{
			robotLifterRetract.Set(true);
			robotLifterExtend.Set(false);
			stinger_toggled = true;
		}
		else if (!(leftStick.GetRawButton(3)  || rightStick.GetRawButton(3)))
		{
			stinger_toggled = false;
		}
		
		if (frisbeeShooter.GetPower() != 0.0)
		{
			compressor.Stop();
		}
		else
		{
			compressor.Start();
		}
		
		//frisbeeShooter.ControlSpeed();
		myRobot.TankDrive(-leftStick.GetY(), -rightStick.GetY()); // drive with arcade style (use right stick)
		
		driverStationLCD->UpdateLCD();
		
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
			default:
			case AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD:
//				autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK;
//				break;
//			case AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK:
				autonomousMode = AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD;
				break;
			case AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD:
				autonomousMode = AUTONOMOUS_MODE_THREE_FRISBEE;
				break;
			case AUTONOMOUS_MODE_THREE_FRISBEE:
				autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD;
				break;
				
//			case AUTONOMOUS_MODE_FIVE_FRISBEE_BACK:
//				autonomousMode = AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD;
//				break;
//			case AUTONOMOUS_MODE_THREE_FRISBEE:
//				autonomousMode = AUTONOMOUS_MODE_TWO_FRISBEE;
//				break;
//			case AUTONOMOUS_MODE_TWO_FRISBEE:
//				autonomousMode = AUTONOMOUS_MODE_FOUR_FRISBEE;
//				break;
//			case AUTONOMOUS_MODE_FOUR_FRISBEE:
//				autonomousMode = AUTONOMOUS_MODE_SIT_AND_SHOOT;
//				break;
//			case AUTONOMOUS_MODE_SIT_AND_SHOOT:
//				autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD;
//				break;
//			
//			case AUTONOMOUS_MODE_FEED_FRISBEE:
//				autonomousMode = AUTONOMOUS_MODE_FIVE_FRISBEE_BACK;
//				break;
			}
		}	

		switch (autonomousMode)
		{
		default:
		case AUTONOMOUS_MODE_FIVE_FRISBEE_BACK:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 5, pickup behind");
				break;
		case AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 5, pickup forward");
				break;
		case AUTONOMOUS_MODE_THREE_FRISBEE:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 3 frisbees");
				break;
		case AUTONOMOUS_MODE_TWO_FRISBEE:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 2 front");
				break;
		case AUTONOMOUS_MODE_FOUR_FRISBEE:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 4 forward");
				break;
		case AUTONOMOUS_MODE_SIT_AND_SHOOT:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot from feeder");
				break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 7, all forward");
				break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 7, forward then back");
				break;
		case AUTONOMOUS_MODE_FEED_FRISBEE:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Feed frisbees to another robot");
				break;
		}

		driverStationLCD->UpdateLCD();
	}

	
		
	void AutonomousPeriodic (void)
	{
		timeInAutonomous.Start();
		switch (autonomousMode)
		{
		case AUTONOMOUS_MODE_FIVE_FRISBEE_BACK:
			break;
		case AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD:
			//Third
			AutonFiveFrisbeeForward();
			break;
		case AUTONOMOUS_MODE_THREE_FRISBEE:
			AutonThreeFrisbee();
			//Second
			break;
		case AUTONOMOUS_MODE_TWO_FRISBEE:
			break;
		case AUTONOMOUS_MODE_FOUR_FRISBEE:
			break;
		case AUTONOMOUS_MODE_SIT_AND_SHOOT:
			break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD:
			AutonSevenFrisbeeForward();
			break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK:
			//AutonSevenFrisbeeBack();
			break;
		default:
		case AUTONOMOUS_MODE_FEED_FRISBEE:
			break;
		}
		if (frisbeeShooter.GetPower() != 0.0)
		{
			compressor.Stop();
		}
		else
		{
			compressor.Start();
		}
		//myRobot.Drive(forwardSpeed, curve); curve less than 0 turns left, curve greater than zero turns right
		autonTurnAmount = gyro.GetAngle() / 100;
		if (autonTurnAmount > 0.1) autonTurnAmount = 0.1;
		if (autonTurnAmount < -0.1) autonTurnAmount = -0.1;
		//autonTurnAmount = 2 * ((leftEncoder.Get() - rightEncoder.Get()) / (leftEncoder.Get() + rightEncoder.Get()));
		//if (autonTurnAmount > 0.15) autonTurnAmount = 0.15;
		//if (autonTurnAmount < -0.15) autonTurnAmount = -0.15;
				
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "Auton Step: %d", autonStepCount);
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "L:%d, R:%d T:%f", leftEncoder.Get(), rightEncoder.Get(), autonTurnAmount);
		driverStationLCD->UpdateLCD();
		
	}
	
	bool AutonomousShoot(int quantity, bool use_sensor, bool reset = false, float delay_time = 0.0)
	{
		if (reset == true)
		{
			autonShotCount = 0;
			autonShooting.Reset();
			autonShooting.Start();
		}
		frisbeeShooter.ShootFrisbee(false);
		
		if(autonShotCount < quantity)
		{
			if (autonShooting.Get() > delay_time/*(use_sensor != true) || limitSwitchLoader.Get() == true*/)
			{
				if (frisbeeShooter.ShootFrisbee(true) == true)
				{
					autonShotCount ++;
					autonShooting.Reset();
					autonShooting.Start();
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
			leftEncoder.Reset();
			rightEncoder.Reset();
			autonTurnAmount = 0;
			autonDrivingBack.Reset();
			autonDrivingBack.Start();
		}
		
		if (autonDrivingBack.Get() > time /*|| (limitSwitchLeft.Get() && limitSwitchRight.Get()) || distance_traveled > distance*/)
		{
			myRobot.Drive(0.0, 0.0);
			return true;//Stop motors and return true
		}
		//Slow down before reaching stop point
		else if (autonDrivingBack.Get()  > (time - 0.5) /*|| (limitSwitchLeft.Get() && limitSwitchRight.Get()) || distance_traveled > distance*/)
		{
			myRobot.Drive(-0.1, autonTurnAmount);
			//myRobot.Drive(-0.75 + (time - autonDrivingBack.Get()), autonTurnAmount);
//			myRobot.TankDrive(-0.75 +(time - autonDrivingBack.Get()) - autonTurnAmount, -0.25 +(time - autonDrivingBack.Get()) + autonTurnAmount);
			if (collector_enabled) collector.Collect();
			return false;//Stop motors and return true
		}
		else if (autonDrivingBack.Get() > 0.25)
		{
			//TODO use the encoder turn rate
//			myRobot.Drive(-0.75, -.1);//Set speed to 50% and collect
			myRobot.Drive(-0.75, autonTurnAmount);//Set speed to 50% and collect
//			myRobot.TankDrive(-0.75 - autonTurnAmount, -0.75 + autonTurnAmount);
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		else
		{
			myRobot.Drive(-0.25 - (autonDrivingBack.Get() * 2.0), autonTurnAmount);//Set speed to 50% and collect
//			myRobot.TankDrive(-0.25 - autonDrivingBack.Get() - autonTurnAmount, -0.25 - autonDrivingBack.Get() + autonTurnAmount);
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		
		return false;
	}
	
	bool AutonomousCollectForward(float distance, float time, bool collector_enabled = true, bool reset = false)
	{
		if (reset == true)
		{
			leftEncoder.Reset();
			rightEncoder.Reset();
			autonTurnAmount = 0;
			autonDrivingForward.Reset();
			autonDrivingForward.Start();
		}

		if (autonDrivingForward.Get()> time /*|| (limitSwitchLeft.Get() && limitSwitchRight.Get()) || distance_traveled > distance*/)
		{
			myRobot.Drive(0.0, 0.0);
			return true;//Stop motors and return true
		}
		else if (autonDrivingForward.Get() > 0.25)
		{
			myRobot.Drive(1.0, -autonTurnAmount);//Set speed to 50% and collect
//			myRobot.TankDrive(0.9 - autonTurnAmount, 0.9 + autonTurnAmount);
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		else
		{
			myRobot.Drive(0.5 + (autonDrivingForward.Get() * 2.0), -autonTurnAmount);//Set speed to 50% and collect
//			myRobot.TankDrive(0.5 - (autonDrivingForward.Get() * 2.0) - autonTurnAmount, 0.5 - (autonDrivingForward.Get() * 2.0) + autonTurnAmount);
			if (collector_enabled) collector.Collect();
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
			if (collector.GetPosition() == Collector::UP)
			{
				collector.Load();
			}
			return true;
		}
		else
		{
			collector.Load();
			return false;
		}
		return false;	
	}
	
	bool AutonomousLowerCollector(void)
	{
		if (collector.GetPosition() == Collector::DOWN)
		{
			collector.Idle();
			return true;
		}
		else
		{
			collector.Collect();
		}
		return false;
	}
	
	bool AutonomousDislodgeCollector(float time, bool reset = false)
	{
		if (reset == true)
		{
			autonDrivingForward.Reset();
			autonDrivingForward.Start();
		}
		if (autonDrivingForward.Get() > time)
		{
			myRobot.Drive(0.0,0.0);
			return true;
		}
		else if (autonDrivingForward.Get() < 0.4)
		{
			myRobot.Drive(0.0,0.0);
		}
		else
		{
			collector.LeaveStartingPosition();
			myRobot.Drive(0.30,0.0);
		}
		return false;
	}
	
	void AutonSevenFrisbeeForward(void)
	{
		bool condition1, condition2;
		frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
		switch (autonStepCount)
		{
		case 0:
			//TODO determine rpm
			frisbeeShooter.SetRpm(4000);
			if (frisbeeShooter.IsReady() == true)
			{
				autonReset = true;
				autonStepCount++;
			}
			
			break;
		case 1: 	//Shoot 3 frisbees (3sec) and lower collector
			if (AutonomousShoot(3,false,autonReset))
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 2:
			condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset); 
			condition2 = AutonomousLowerCollector();
			if (condition1 && condition2)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 3: 		//Drive forward and collect
			if (AutonomousCollectForward(999.0,1.0,true,autonReset)== true)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
	
			break;
		case 4:	//Load frisbees
			myRobot.Drive(0.25,autonTurnAmount);
			if (AutonomousLoadFrisbees(true,autonReset) )
			{
				if (AutonomousLowerCollector())
				{
					autonReset = true;
					autonStepCount++;
				}
			}
			else
			{
				autonReset = false;
			}
			break;
		case 5:	//Drive forward and lower collector
			if (AutonomousCollectForward(999.0,0.5,true,autonReset) == true)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 6:	//Stop motor and begin loading
			myRobot.Drive(0.0,0.0);
			AutonomousLoadFrisbees(true,autonReset);
			if (autonLoading.Get() > 0.25)
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
			//Load and Drive Backward without collecting
			condition1 = AutonomousCollectBack(999.0,1.0,false,autonReset);
			condition2 = AutonomousLoadFrisbees(true,false);
			if (condition1 && condition2)
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
			if (autonReset)
			{
				autonDrivingForward.Reset();
				autonDrivingForward.Start();
			}
			//tilt up to shoot frisbees
			myRobot.Drive(0.0,0.0);
			robotLifterExtend.Set(true);
			robotLifterRetract.Set(false);
			if (autonDrivingForward.Get() >=  0.75)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 9:
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
		case 10:

			robotLifterExtend.Set(false);
			robotLifterRetract.Set(true);
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
			timeInAutonomous.Stop();
			autonReset = true;
			autonStepCount++;
			break;
		case 11:
			autonReset = true;
			break;
				
		}
				
	}

	void AutonThreeFrisbee(void)
	{
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
			frisbeeShooter.SetPower(POWER_AUTONOMOUS_SHOTS);

			if (autonDrivingForward.Get() > TIME_AUTONOMOUS_SPIN_UP)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 1: 	//Shoot 3 frisbees (3sec) and lower collector
			if (AutonomousShoot(3,true,autonReset, 1.5) == true)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;

		case 2:
			
			if ((AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset) == true) && 
					(AutonomousLowerCollector() == true))
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 3:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
			timeInAutonomous.Stop();
			autonReset = true;
			autonStepCount++;
			break;
		case 4:
			autonReset = true;
			break;
		}
	}
		//TODO
	void AutonFiveFrisbeeForward(void)
	{
		bool condition1, condition2;
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
			
			frisbeeShooter.SetPower(POWER_AUTONOMOUS_SHOTS);

			if (autonDrivingForward.Get() > TIME_AUTONOMOUS_SPIN_UP)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 1: 	//Shoot 3 frisbees (3sec) and lower collector
			if (AutonomousShoot(3,true,autonReset, 1.35))
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 2:
			condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset);
			condition2 = AutonomousLowerCollector();
			
			if (condition1 && condition2)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 3: 		//Drive forward and collect
			if (AutonomousCollectForward(999.0,1.0,true,autonReset)== true)
			{
				myRobot.Drive(0.0,0.0);
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
	
			break;
		case 4:	//Load frisbees and drive back
			condition1 = AutonomousLoadFrisbees(true,autonReset);
			condition2 = AutonomousCollectBack(999.0,1.4,false,autonReset);
			if (condition1 && condition2)
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 5:	//Wait .1 sec
			if (autonDrivingBack.Get() > 1.5)
			{
				autonReset = true;
				autonStepCount++;
			}
			break;
		case 6:
			//Shoot 2 frisbees (2 seconds)
			if (AutonomousShoot(2,true,autonReset, 1.0))
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
			frisbeeShooter.SetPower(0);
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
			timeInAutonomous.Stop();
			autonReset = true;
			autonStepCount++;
			break;
		case 8:
			autonReset = true;
			break;
		case 9:
			break;
		}		
	}
};

START_ROBOT_CLASS(RobotDemo);

