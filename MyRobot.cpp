#include "WPILib.h"
#include "Shooter.h"
#include "TargetCamera.h"
#include "Collector.h"
#include "Donuts"
//#include "math.h"

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
#define RPM_AUTONOMOUS_SHOTS 3850
#define RPM_TELEOP_SHOTS 4000
#define RPM_FIVE_POINTER     2625

#define AUTON_STRAIGHTEN() myRobot.TankDrive((autonTurnAmount * -4.0), (autonTurnAmount * 4.0))
#define AUTON_SPEED_CORRECT_FACTOR 0.5

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
    SOLENOID_SHOOTER_RETRACT
}SOLENOID_CHANNEL;

typedef enum
{
    SOLENOID2_0,
    SOLENOID_CLIMBER_EXTEND,
    SOLENOID_CLIMBER_RETRACT,
    SOLENOID_EXTRA_COLLECTOR_EXTEND,
    SOLENOID_EXTRA_COLLECTOR_RETRACT,
    SOLENOID_FRISBEE_UNJAM_EXTEND,
    SOLENOID_FRISBEE_UNJAM_RETRACT
    
}SOLENOID_CHANNEL2;

typedef enum
{
    DIGITAL_0,
    DIGITAL_COMPRESSOR_SWITCH,
    DIGITAL_WHEEL_RIGHT_FORWARD,
    DIGITAL_WHEEL_RIGHT_BACKWARD,
    DIGITAL_WHEEL_LEFT_FORWARD,
    DIGITAL_WHEEL_LEFT_BACKWARD,
    DIGITAL_LEFT_COLLECTOR_SWITCH,
    DIGITAL_SHOOTER_SENSOR,
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
	Solenoid robotClimberExtend;
	Solenoid robotClimberRetract;
	Solenoid frisbeeUnjamExtend;
	Solenoid frisbeeUnjamRetract;
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
	float autonSpeedCorrect;
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
		robotClimberExtend(2, SOLENOID_CLIMBER_EXTEND),
		robotClimberRetract(2, SOLENOID_CLIMBER_RETRACT),
		frisbeeUnjamExtend(2, SOLENOID_FRISBEE_UNJAM_EXTEND),
		frisbeeUnjamRetract(2, SOLENOID_FRISBEE_UNJAM_RETRACT),
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
		autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD;
		compressor.Start();
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
		autonSpeedCorrect = 0;
		gyro.Reset();
		leftBrake.Set(false);
		rightBrake.Set(false);
	}

	void TeleopPeriodic (void)
	{
		static bool stinger_toggled = false;
		static bool climber_toggled = false;
	    compressor.Start();
		//targetCamera.SetDebugMode(operatorStick.GetRawButton(9));
	    float shot_power = 0;
	    float rpm = frisbeeShooter.GetRpm();
		static bool previous_shoot_state = false;
	//Test code
	    float shot_rpm = 0.0;

	    if ((operatorStick.GetRawButton(1)==true)||(leftStick.GetRawButton(1)==true))
		{
			if (operatorStick.GetRawButton(10)==true)
			{
				shot_rpm = 4400.0 * ((1-operatorStick.GetThrottle())/2);
			}
			else if (robotLifterExtend.Get() == true)
			{
				shot_rpm = RPM_FIVE_POINTER;
			}
			else
			{
				shot_rpm = RPM_TELEOP_SHOTS;
			}
		}

	    frisbeeShooter.SetRpm(shot_rpm);
	    driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "Power: %f", frisbeeShooter.GetPower());	
	    driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Rpm: %f %d", rpm, shooterSensor.Get());
	    driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "TRpm: %f", shot_rpm);			
		
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
		else if (operatorStick.GetRawButton(5) == true)
		{
			collector.Feed(true);
		}
		else 
		{
			//stop collector
			collector.Idle();
		}
		
		if ((operatorStick.GetRawButton(11)==true )||(rightStick.GetRawButton(1) == true))
		{
			//shoot frisbees
			if ((robotLifterExtend.Get() == false) || (previous_shoot_state == false))
			{
				previous_shoot_state = frisbeeShooter.ShootFrisbee(true);
			}
			else
			{
				frisbeeShooter.ShootFrisbee(false);
			}
		}
		else
		{
			previous_shoot_state = false;
			frisbeeShooter.ShootFrisbee(false);
		}

	    driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "%d   %d", previous_shoot_state, frisbeeShooter.IsReady());			

	    //if left 5 and right 4 and climber_toggled = false, then allow the climber to toggle
	    if (leftStick.GetRawButton(5) && rightStick.GetRawButton(4) && (climber_toggled == false))
	    {
	    	if (robotClimberExtend.Get() == false)
	    	{
				robotClimberExtend.Set(true);
				robotClimberRetract.Set(false);
	    	}
	    	else
	    	{
		    	robotClimberExtend.Set(false);
		    	robotClimberRetract.Set(true);
	    	}
	    	climber_toggled = true;
	    }
	    //If a button gets let go, set toggled to false so it can be pressed again
	    else if (!(leftStick.GetRawButton(5) || rightStick.GetRawButton(4)))
	    {
	    	climber_toggled = false;
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
		
		if (operatorStick.GetRawButton(6))
		{
			frisbeeUnjamRetract.Set(false);
			frisbeeUnjamExtend.Set(true);
		}
		else 
		{
			frisbeeUnjamRetract.Set(true);
			frisbeeUnjamExtend.Set(false);
		}
		
		static Timer start_position_timer;
		
		if (operatorStick.GetRawButton(9) == true)
		{
			start_position_timer.Start();
			if (start_position_timer.Get() > 2.0)
			{
				collector.EnterStartingPosition();
			}
			else
			{
				collector.LeaveStartingPosition();
			}
		}
		else
		{
			start_position_timer.Reset();
			start_position_timer.Stop();
		}
		
		
		
//		if (frisbeeShooter.GetPower() != 0.0)
//		{
//			compressor.Stop();
//		}
//		else
//		{
//			compressor.Start();
//		}
		
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
//		if (frisbeeShooter.GetPower() != 0.0)
//		{
//			compressor.Stop();
//		}
//		else
//		{
//			compressor.Start();
//		}
		//myRobot.Drive(forwardSpeed, curve); curve less than 0 turns left, curve greater than zero turns right
		autonTurnAmount = gyro.GetAngle() / 100;
		if (autonTurnAmount > 0.1) autonTurnAmount = 0.1;
		if (autonTurnAmount < -0.1) autonTurnAmount = -0.1;
		
		//autonSpeedCorrect = fabsf(autonTurnAmount) * AUTON_SPEED_CORRECT_FACTOR;
		
		autonSpeedCorrect = (autonTurnAmount) * AUTON_SPEED_CORRECT_FACTOR;
		if (autonSpeedCorrect < 0.0) autonSpeedCorrect = autonSpeedCorrect * -1.0;
		
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
			autonSpeedCorrect = 0;
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
			myRobot.Drive(-(0.1 + autonSpeedCorrect) , autonTurnAmount);
			if (collector_enabled) collector.Collect();
			return false;//Stop motors and return true
		}
		else if (autonDrivingBack.Get() > 0.25)
		{
			//TODO use the encoder turn rate
			myRobot.Drive(-(0.40 + autonSpeedCorrect), autonTurnAmount);//Set speed to 50% and collect
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		else
		{
			myRobot.Drive(-(0.25 + autonSpeedCorrect) - (autonDrivingBack.Get() * 2.0), autonTurnAmount);//Set speed to 50% and collect
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		
		return false;
	}

	bool AutonomousCollectBackFast(float distance, float time, bool collector_enabled = true, bool reset = false)
	{
		if (reset == true)
		{
			leftEncoder.Reset();
			rightEncoder.Reset();
			autonTurnAmount = 0;
			autonSpeedCorrect = 0;
			autonDrivingBack.Reset();
			autonDrivingBack.Start();
		}
		
		if (autonDrivingBack.Get() > time /*|| (limitSwitchLeft.Get() && limitSwitchRight.Get()) || distance_traveled > distance*/)
		{
			myRobot.Drive(0.0, 0.0);
			return true;//Stop motors and return true
		}
		else if (autonDrivingBack.Get() > 0.25)
		{
			//TODO use the encoder turn rate
			myRobot.Drive(-(0.70 + autonSpeedCorrect), autonTurnAmount);//Set speed to 50% and collect
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		else
		{
			myRobot.Drive(-(0.25 + autonSpeedCorrect) - (autonDrivingBack.Get() * 2.0), autonTurnAmount);//Set speed to 50% and collect
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		
		return false;
	}
	
	bool AutonomousCollectBackReallyFast(float distance, float time, bool collector_enabled = true, bool reset = false)
	{
		if (reset == true)
		{
			leftEncoder.Reset();
			rightEncoder.Reset();
			autonTurnAmount = 0;
			autonSpeedCorrect = 0;
			autonDrivingBack.Reset();
			autonDrivingBack.Start();
		}
		
		if (autonDrivingBack.Get() > time /*|| (limitSwitchLeft.Get() && limitSwitchRight.Get()) || distance_traveled > distance*/)
		{
			myRobot.Drive(0.0, 0.0);
			return true;//Stop motors and return true
		}
		else if (autonDrivingBack.Get() > 0.25)
		{
			//TODO use the encoder turn rate
			myRobot.Drive(-(1.00 + autonSpeedCorrect), autonTurnAmount);//Set speed to 50% and collect
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		else
		{
			myRobot.Drive(-(0.25 + autonSpeedCorrect) - (autonDrivingBack.Get() * 2.0), autonTurnAmount);//Set speed to 50% and collect
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		
		return false;
	}
	
	bool AutonomousCollectForward(float distance, float time, bool collector_enabled = true, bool reset = false, bool stop_at_end = true)
	{
		if (reset == true)
		{
			leftEncoder.Reset();
			rightEncoder.Reset();
			autonTurnAmount = 0;
			autonSpeedCorrect = 0;
			autonDrivingForward.Reset();
			autonDrivingForward.Start();
		}

		if (autonDrivingForward.Get()> time /*|| (limitSwitchLeft.Get() && limitSwitchRight.Get()) || distance_traveled > distance*/)
		{
			if (stop_at_end)
			{
				myRobot.Drive(0.0, 0.0);
			}
			else
			{
				myRobot.Drive(0.25 + autonSpeedCorrect, -autonTurnAmount);
			}
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
			myRobot.Drive(0.5 + autonSpeedCorrect + (autonDrivingForward.Get() * 2.0), -autonTurnAmount);//Set speed to 50% and collect
//			myRobot.TankDrive(0.5 - (autonDrivingForward.Get() * 2.0) - autonTurnAmount, 0.5 - (autonDrivingForward.Get() * 2.0) + autonTurnAmount);
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		
		return false;
	}
	
	bool AutonomousLoadFrisbees(bool use_switch = true, bool reset = false, float time = 3.0)
	{
		if (reset == true)
		{
			autonLoading.Reset();
			autonLoading.Start();
		}
		
		if ((autonLoading.Get() > time) /*|| (limitSwitchLoader.Get() && use_switch)*/)
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
	
	void AutonSevenFrisbeeForwardWithLifters(void)
	{
		bool condition1, condition2;
		frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
		switch (autonStepCount)
		{
		case 0:
			//TODO determine rpm
			frisbeeShooter.SetRpm(RPM_AUTONOMOUS_SHOTS);
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
			if (AutonomousCollectForward(999.0,1.1,true,autonReset)== true)
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
			myRobot.Drive(0.20 + autonSpeedCorrect,-autonTurnAmount);
			if (AutonomousLoadFrisbees(true,autonReset,2.0) )
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
			if (AutonomousCollectForward(999.0,0.65,true,autonReset) == true)
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
			condition1 = AutonomousCollectBack(999.0,1.8,false,autonReset);
			condition2 = AutonomousLoadFrisbees(true,false, 1.5);
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
			robotLifterExtend.Set(true);
			robotLifterRetract.Set(false);
			if (autonDrivingForward.Get() < 0.5)
			{
				AUTON_STRAIGHTEN();
			}
			else
			{
				myRobot.Drive(0.0,0.0);
			}
			
			if (autonDrivingForward.Get() >=  1.0)
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
			if (AutonomousShoot(8,true,autonReset))
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
			//frisbeeShooter.SetPower(POWER_AUTONOMOUS_SHOTS);
			frisbeeShooter.SetRpm(RPM_AUTONOMOUS_SHOTS);
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
		
	void AutonFiveFrisbeeForward(void)
	{
		bool condition1, condition2;
		frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
		switch (autonStepCount)
		{

		case 0: 	//Shoot 3 frisbees (3sec) and lower collector

			frisbeeShooter.SetRpm(RPM_AUTONOMOUS_SHOTS);
			if (AutonomousShoot(3,true,autonReset, 0.5))
			{
				autonReset = true;
				autonStepCount++;
			}
			else
			{
				autonReset = false;
			}
			break;
		case 1:
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
		case 2: 		//Drive forward and collect
			if (AutonomousCollectForward(999.0,1.05,true,autonReset)== true)
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
		case 3:	//Load frisbees and drive back
			condition1 = AutonomousLoadFrisbees(true,autonReset);
			condition2 = AutonomousCollectBack(999.0,1.8,false,autonReset);
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
		case 4:	//Wait .1 sec

			if (autonReset)
			{
				autonDrivingForward.Reset();
				autonDrivingForward.Start();
			}
			
			//tilt up to shoot frisbees
			if ((autonDrivingForward.Get() < 0.65) && (autonDrivingForward.Get() > 0.15))
			{
				AUTON_STRAIGHTEN();
			}
			else
			{
				myRobot.Drive(0.0,0.0);
			}
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
		case 5:
			//Shoot 2 frisbees (2 seconds)
			if (AutonomousShoot(6,true,autonReset,0.5))
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
			frisbeeShooter.SetRpm(0);
			collector.Idle();
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
			timeInAutonomous.Stop();
			autonReset = true;
			autonStepCount++;
			break;
		case 7:
			autonReset = true;
			break;
		case 8:
			break;
		}		
	}


	
	
	void AutonNineFrisbee(void)
		{
			bool condition1, condition2;
			frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
			switch (autonStepCount)
			{
			case 0:
				//TODO determine rpm
				frisbeeShooter.SetRpm(RPM_AUTONOMOUS_SHOTS);
				if (frisbeeShooter.IsReady() == true)
				{
					autonReset = true;
					autonStepCount++;
				}
				
				break;
			case 1: 	//Shoot 3 frisbees (3sec) and lower collector

				//TODO condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset); 
				condition2 = AutonomousShoot(3,false,autonReset);
				if (condition2)
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
				condition2 = collector.Lower();
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
				if (AutonomousCollectForward(999.0,1.25,true,autonReset)== true)
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
				myRobot.Drive(0.20 + autonSpeedCorrect,-autonTurnAmount);
				if (AutonomousLoadFrisbees(true,autonReset,1.5) )
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
				if (AutonomousCollectForward(999.0,0.75,true,autonReset) == true)
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
				AutonomousCollectBackReallyFast(999.0,2.6,false,autonReset);
				
				if (AutonomousLoadFrisbees(true,false, 1.5))
				{
					collector.Lower();
				}
				if (autonDrivingBack.Get() > 1.4)
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
				//Shoot 2 while driving (like a boss)

				AutonomousCollectBackReallyFast(999.0,2.6,false,false);
				if (AutonomousShoot(2,true,autonReset))
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

				if (AutonomousCollectBackReallyFast(999.0,2.6,true, false))
				{
					autonReset = true;
					autonStepCount++;
				}
				else
				{
					autonReset = false;
				}
			case 10:
				condition1 = AutonomousLoadFrisbees(false, autonReset, 1.5);
				condition2 = AutonomousShoot(2,false,autonReset);
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
			case 11:
				if (AutonomousShoot(2,false,autonReset))
				{
					autonReset = true;
					autonStepCount++;
				}
				else
				{
					autonReset = false;
				}
				break;
			case 12:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
				timeInAutonomous.Stop();
				autonReset = true;
				autonStepCount++;
				break;
			case 13:
				autonReset = true;
				break;
					
			}
					
		}	
	
	
	
	
	
	
	
	

	void AutonSevenFrisbeeForward(void)
		{
			bool condition1, condition2;
			frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
			switch (autonStepCount)
			{
			case 0:
				//TODO determine rpm
				frisbeeShooter.SetRpm(RPM_AUTONOMOUS_SHOTS);
				if (frisbeeShooter.IsReady() == true)
				{
					autonReset = true;
					autonStepCount++;
				}
				
				break;
			case 1: 	//Shoot 3 frisbees (3sec) and lower collector

				//condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset); 
				condition2 = AutonomousShoot(3,false,autonReset);
				if (condition2)
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
				myRobot.Drive(0.20 + autonSpeedCorrect,-autonTurnAmount);
				if (AutonomousLoadFrisbees(true,autonReset,2.0) )
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
				if (AutonomousCollectForward(999.0,0.6,true,autonReset) == true)
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
				condition1 = AutonomousCollectBackFast(999.0,2.5,false,autonReset);
				condition2 = AutonomousLoadFrisbees(true,false, 2.0);
				if (condition2)
				{
					collector.Lower();
				}
				
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
				//Shoot 4 frisbees (4 seconds)
				if (AutonomousShoot(8,true,autonReset))
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

				driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
				timeInAutonomous.Stop();
				autonReset = true;
				autonStepCount++;
				break;
			case 10:
				autonReset = true;
				break;
					
			}
					
		}
	
	
	
	

};

START_ROBOT_CLASS(RobotDemo);

