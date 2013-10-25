#include "WPILib.h"
#include "Shooter.h"
#include "TargetCamera.h"
#include "Collector.h"
#include "Donuts"
#include "customPIDs.h"
//#include "math.h"


//7shot backup parameters, change time by .1 each try, change speed by .05 each try if the robot is still rolling when it shoots
#define SEVEN_SHOT_BACKUP_TIME 2.95
#define SEVEN_SHOT_BACKUP_SPEED 0.70



#define TIME_AUTONOMOUS_DISLODGE 0.3
#define TIME_AUTONOMOUS_SPIN_UP 0.5
#define POWER_AUTONOMOUS_SHOTS 0.56
#define RPM_AUTONOMOUS_FIRST_SHOTS 4075
#define RPM_AUTONOMOUS_LAST_SHOT RPM_AUTONOMOUS_LAST_FOUR
#define RPM_AUTONOMOUS_LAST_FOUR 4075
#define RPM_AUTONOMOUS_CORNER_SHOTS 4000
//#define RPM_AUTONOMOUS_CORNER_SHOTS 4025 //left side
#define RPM_TELEOP_SHOTS 4000
#define RPM_LAST_TELEOP_SHOT 4000
#define RPM_FIVE_POINTER     2625

#define AUTON_STRAIGHTEN() myRobot.TankDrive((autonTurnAmount * -4.0), (autonTurnAmount * 4.0))
#define AUTON_SPEED_CORRECT_FACTOR 0.5


//PID Parameters
#define ROTATION_PID_PROPORTION 0.061584
#define ROTATION_PID_INTEGRAL 0.0
#define ROTATION_PID_DERIVATIVE 0.055718
#define ROTATION_PID_SPEED_OFFSET 0.283465

#define ROTATION_PID_MIN_INPUT -180.0
#define ROTATION_PID_MAX_INPUT 180.0
#define ROTATION_PID_MIN_OUTPUT -1.00
#define ROTATION_PID_MAX_OUTPUT 1.00

#define ROTATION_PID_TOLERENCE 0.50
#define ROTATION_PID_SETPOINT_OFFSET 0.0 //negative adjusts to the right


typedef enum
{
    PWM_0,
    PWM_LEFT_DRIVE_FRONT,
    PWM_LEFT_DRIVE_BACK,
    PWM_SHOOTER_WHEEL,
    PWM_FAKE_1,
    PWM_RIGHT_DRIVE_BACK,
    PWM_BACK_COLLECTOR,
    PWM_FRONT_COLLECTOR,
    PWM_RIGHT_DRIVE_FRONT,
    PWM_FAKE_2,
    PWM_6_SERVO_TEST
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
    DIGITAL_WHEEL_RIGHT_FORWARD,
    DIGITAL_WHEEL_RIGHT_BACKWARD,
    DIGITAL_WHEEL_LEFT_FORWARD,
    DIGITAL_WHEEL_LEFT_BACKWARD,
    DIGITAL_LEFT_COLLECTOR_SWITCH,
    DIGITAL_SHOOTER_SENSOR,
    DIGITAL_COMPRESSOR_SWITCH,
    DIGITAL_RIGHT_COLLECTOR_SWITCH,
    DIGITAL_HOPPER_SWITCH,
    DIGITAL_LEFT_BRAKE_OUT,
    DIGITAL_RIGHT_BRAKE_OUT
        
}DIGITAL_IO_CHANNEL;
typedef enum
{
    RELAY_0,
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
	AUTONOMOUS_MODE_THREE_FRISBEE_CORNER,
	AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD,
	AUTONOMOUS_MODE_SIT_AND_SHOOT
}AUTONOMOUS_MODE_SELECT;

AUTONOMOUS_MODE_SELECT autonomousMode;


class RobotDemo : public IterativeRobot
{
#if (1)
    Talon leftFrontMotor;
    Talon leftRearMotor;
    Talon rightFrontMotor;
    Talon rightRearMotor;
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
	Servo collectorServo;
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
	Timer autonDislodge;
	Timer teleopTime;
	Timer gyroOffTargetTimer;
	bool teleopInput;
	Encoder rightEncoder;
	Encoder leftEncoder;
	GyroControlledTurning rotationControl;
	PIDController rotationPID;
	int autonShotCount;
	int autonStepCount;
	float autonTurnAmount;
	float autonSpeedCorrect;
	bool autonReset;
	bool continueAuton;
#endif
public:
	RobotDemo(void): //initialized
	    leftFrontMotor(PWM_LEFT_DRIVE_FRONT),
	    leftRearMotor(PWM_LEFT_DRIVE_BACK),
	    rightFrontMotor(PWM_RIGHT_DRIVE_FRONT),
	    rightRearMotor(PWM_RIGHT_DRIVE_BACK),
		myRobot(leftFrontMotor,leftRearMotor,rightFrontMotor,rightRearMotor),	// these must be initialized in the same order
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
		collectorServo(PWM_6_SERVO_TEST),
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
        leftEncoder(DIGITAL_WHEEL_LEFT_FORWARD, DIGITAL_WHEEL_LEFT_BACKWARD),
		rotationControl(&myRobot),
		rotationPID(ROTATION_PID_PROPORTION, ROTATION_PID_INTEGRAL, ROTATION_PID_DERIVATIVE, &gyro, &rotationControl)
		
	{
		// Acquire the Driver Station object
		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();
		
		myRobot.SetExpiration(0.1);

		rotationPID.SetInputRange(ROTATION_PID_MIN_INPUT,ROTATION_PID_MAX_INPUT);
		rotationPID.SetOutputRange(ROTATION_PID_MIN_OUTPUT,ROTATION_PID_MAX_OUTPUT);
		rotationPID.SetAbsoluteTolerance(ROTATION_PID_TOLERENCE);
		rotationControl.SetOffset(ROTATION_PID_SPEED_OFFSET);
		rotationPID.Disable();
		
		autonReset = true;
	}

	void DisabledInit(void)
	{
		frisbeeUnjamExtend.Set(false);
		frisbeeUnjamRetract.Set(true);
		leftBrake.Set(true);
		rightBrake.Set(true);
		autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD;
		compressor.Stop();
		rotationPID.Disable();
		continueAuton = false;
	}
	
	void TeleopInit (void)
	{
	    
		myRobot.SetSafetyEnabled(true);
	    compressor.Start();
	    leftBrake.Set(true);
	    rightBrake.Set(true);
	    
	    collector.LeaveStartingPosition();
	    teleopTime.Reset();
	    teleopTime.Start();
	    teleopInput = false;
	}
	
	void AutonomousInit (void)
	{
	    continueAuton = true;
		myRobot.SetSafetyEnabled(false);
	    collector.EnterStartingPosition();
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
		
        rotationPID.SetInputRange(ROTATION_PID_MIN_INPUT,ROTATION_PID_MAX_INPUT);
        rotationPID.SetOutputRange(ROTATION_PID_MIN_OUTPUT,ROTATION_PID_MAX_OUTPUT);
        rotationPID.SetAbsoluteTolerance(ROTATION_PID_TOLERENCE);
        rotationPID.Disable();

        gyroOffTargetTimer.Reset();
	}
	
	void TeleopPeriodic (void)
	{

//        float gyro_angle = gyro.GetAngle();
//        driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "angle:%f", gyro_angle);
//        driverStationLCD->UpdateLCD();
	    
		static bool climber_toggled = false;
	    compressor.Start();
	    float rpm = frisbeeShooter.GetRpm();
		static bool previous_shoot_state = false;
		static int shot_count = 0;
		static Timer shot_timer;
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
			else if (shot_count >= 3)
			{
			    shot_rpm = RPM_LAST_TELEOP_SHOT;
			}
			else
			{
				shot_rpm = RPM_TELEOP_SHOTS;
			}
		}

	    frisbeeShooter.SetRpm(shot_rpm);
	    if (shot_count == 0)
	        driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Rpm: %f", rpm);
	    driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "TRpm: %f", shot_rpm);	
	   
		//Collector controls
		if ((operatorStick.GetRawButton(2)==true )||(leftStick.GetRawButton(2)==true))
		{   
			//lower collector and pick up from floor
			collector.Collect();
		}
		else if ((operatorStick.GetRawButton(4)==true )||(rightStick.GetRawButton(2)==true))
		{
		    //raise collector and load frisbees
			collector.Load();
		}		
		else if ((operatorStick.GetRawButton(3)==true) || (leftStick.GetRawButton(7) == true))
		{
			//barf frisbees backwards
			collector.Feed();
		}
		else if ((operatorStick.GetRawButton(5) == true) || (leftStick.GetRawButton(6) == true))
		{
            //barf frisbees forwards
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
				if (previous_shoot_state == true)
				{
				    shot_count++;
                    driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Rpm: %f", rpm);
				}
				
				shot_timer.Reset();
				shot_timer.Start();
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
		
		if (shot_timer.Get() >= 0.5)
		    shot_count = 0;
		
		
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
	    
	    if (leftStick.GetRawButton(3))
	    {
	        robotClimberExtend.Set(false);
	        robotClimberRetract.Set(true);
	    }
	    else if (rightStick.GetRawButton(3))
	    {
	        robotClimberExtend.Set(true);
	        robotClimberRetract.Set(false);
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
		
		if(leftStick.GetRawButton(8))
		{
		    rightRearMotor.Set(1.0);   
		}
		else if (leftStick.GetRawButton(9))
		{
		    rightFrontMotor.Set(1.0);
		}
		else
		{
		    myRobot.TankDrive(-leftStick.GetY(), -rightStick.GetY());       // drive with arcade style (use right stick)
		}

		driverStationLCD->UpdateLCD();
		
	}

	void DisabledPeriodic(void)
	{
	    float gyro_angle = gyro.GetAngle();
        driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "angle:%f", gyro_angle);
        driverStationLCD->UpdateLCD();
		static Timer button_combo_timer;
		
		if (rightStick.GetRawButton(2) == true,
			leftStick.GetRawButton(2) == true)
		{
			button_combo_timer.Start();
			gyro.Reset();
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
				autonomousMode = AUTONOMOUS_MODE_THREE_FRISBEE_CORNER;
				break;
			case AUTONOMOUS_MODE_THREE_FRISBEE_CORNER:
			    autonomousMode = AUTONOMOUS_MODE_SIT_AND_SHOOT;
			    break;
			case AUTONOMOUS_MODE_SIT_AND_SHOOT:
                autonomousMode = AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD;
			    break;
			}
		}	

		switch (autonomousMode)
		{
		default:
		case AUTONOMOUS_MODE_THREE_FRISBEE_CORNER:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Corner 3");
				break;
		case AUTONOMOUS_MODE_SIT_AND_SHOOT:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Center 3");
				break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Shoot 7, all forward");
				break;
		}

		driverStationLCD->UpdateLCD();
	}
		
	void AutonomousPeriodic (void)
	{
	    float gyro_angle = gyro.GetAngle();
        driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "angle:%f", gyro_angle);
	    if ((gyro_angle > 90.0) || (gyro_angle < -90.0) || (gyroOffTargetTimer.Get() > 1.0))
	    {
	        myRobot.Drive(0.0,0.0);
	        driverStationLCD->UpdateLCD();
	        if (autonStepCount >= 2)        
	            return; 
	    }
	    else if ((gyro_angle > 30.0) || (gyro_angle < -30.0))
	    {
	        gyroOffTargetTimer.Start();
	    }
	    else
	    {
	        gyroOffTargetTimer.Reset();
	    }
	    
		timeInAutonomous.Start();
		switch (autonomousMode)
		{
		case AUTONOMOUS_MODE_THREE_FRISBEE_CORNER:
			AutonThreeFrisbeeCorner();
			break;
		case AUTONOMOUS_MODE_SIT_AND_SHOOT:
		    AutonThreeFrisbeeCenter();
			break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD:
			AutonSevenFrisbeeForward();
			break;
		}
		
		//myRobot.Drive(forwardSpeed, curve); curve less than 0 turns left, curve greater than zero turns right
		autonTurnAmount = gyro_angle / 50.0f;
		if (autonTurnAmount > 0.2) autonTurnAmount = 0.2;
		if (autonTurnAmount < -0.2) autonTurnAmount = -0.2;
		
		autonSpeedCorrect = (autonTurnAmount) * AUTON_SPEED_CORRECT_FACTOR;
		if (autonSpeedCorrect < 0.0) autonSpeedCorrect = autonSpeedCorrect * -1.0;
		
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "Auton Step: %d", autonStepCount);
		driverStationLCD->UpdateLCD();
		
	}
	
	bool AutonomousShoot(int quantity, bool delay_after, bool reset = false, float delay_time = 0.0, float last_shot_rpm = 0.0)
	{
		bool fast_shooting = false;

		if (reset == true)
		{
			autonShotCount = 0;
			autonShooting.Reset();
			autonShooting.Start();
		}
		frisbeeShooter.ShootFrisbee(false, fast_shooting);
		
		if(autonShotCount < quantity)
		{
			if (autonShooting.Get() > delay_time/*(use_sensor != true) || limitSwitchLoader.Get() == true*/)
			{
				if (frisbeeShooter.ShootFrisbee(true, fast_shooting) == true)
				{
					autonShotCount ++;
					autonShooting.Reset();
					autonShooting.Start();
					if ((autonShotCount == (quantity - 1)) && (last_shot_rpm != 0.0))
					{
					    frisbeeShooter.SetRpm(last_shot_rpm);
					}
				}
			}
			return false;
		}
		else if ((delay_after == true) && (autonShooting.Get() < 0.4))
		{
		    
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
			myRobot.Drive(-(SEVEN_SHOT_BACKUP_SPEED + autonSpeedCorrect), autonTurnAmount);//Set speed to 50% and collect
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		else
		{
			myRobot.Drive(-(0.35 + autonSpeedCorrect) - (autonDrivingBack.Get() * 2.0), autonTurnAmount);//Set speed to 50% and collect
			if (collector_enabled) collector.Collect();
			return false;//Return false
		}
		
		return false;
	}
	
	bool AutonomousCollectBackReallyFast(float distance, float time, bool collector_enabled = true, bool reset = false, bool ramped_start = true)
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
		else if ((autonDrivingBack.Get() > 0.25) || (ramped_start == false))
		{
			//TODO use the encoder turn rate
			myRobot.Drive(-1.0, autonTurnAmount);//Set speed to 50% and collect
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
	
    bool AutonomousCollectForward(float distance, float time, bool collector_enabled = true, bool reset = false, bool stop_at_end = true, bool ramped_start = true)
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
        else if ((autonDrivingForward.Get() > 0.25)  || (ramped_start == false))
        {
            myRobot.Drive(0.8, -autonTurnAmount);//Set speed to 50% and collect
//          myRobot.TankDrive(0.9 - autonTurnAmount, 0.9 + autonTurnAmount);
            if (collector_enabled) collector.Collect();
            return false;//Return false
        }
        else
        {
            myRobot.Drive(0.4 + autonSpeedCorrect + (autonDrivingForward.Get() * 2.0), -autonTurnAmount);//Set speed to 50% and collect
//          myRobot.TankDrive(0.5 - (autonDrivingForward.Get() * 2.0) - autonTurnAmount, 0.5 - (autonDrivingForward.Get() * 2.0) + autonTurnAmount);
            if (collector_enabled) collector.Collect();
            return false;//Return false
        }
        
        return false;
    }
   
    bool AutonomousCollectForwardFast(float distance, float time, bool collector_enabled = true, bool reset = false, bool stop_at_end = true, bool ramped_start = true)
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
        else if ((autonDrivingForward.Get() > 0.25)  || (ramped_start == false))
        {
            myRobot.Drive(1.0, -autonTurnAmount);//Set speed to 50% and collect
//          myRobot.TankDrive(0.9 - autonTurnAmount, 0.9 + autonTurnAmount);
            if (collector_enabled) collector.Collect();
            return false;//Return false
        }
        else
        {
            myRobot.Drive(0.5 + autonSpeedCorrect + (autonDrivingForward.Get() * 2.0), -autonTurnAmount);//Set speed to 50% and collect
//          myRobot.TankDrive(0.5 - (autonDrivingForward.Get() * 2.0) - autonTurnAmount, 0.5 - (autonDrivingForward.Get() * 2.0) + autonTurnAmount);
            if (collector_enabled) collector.Collect();
            return false;//Return false
        }
        
        return false;
    }
	
	bool AutonomousLoadFrisbees(bool use_switch = true, bool reset = false, float time = 3.0, float delay = 0.0,float load_speed = 0.0)
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
				collector.Load(load_speed);
			}
			return true;
		}
		else if (autonLoading.Get() >= delay)
		{
			collector.Load(load_speed);
			return false;
		}
		else
		{
			collector.Idle();
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
	
	bool AutonomousDislodgeCollector(float time, bool reset = false, bool stop_at_end = true)
	{
		if (reset == true)
		{
			autonDrivingForward.Reset();
			autonDrivingForward.Start();
		}
		if (autonDrivingForward.Get() > time)
		{
			if (stop_at_end == true)
				myRobot.Drive(0.0,0.0);
			return true;
		}
		else
		{
			collector.LeaveStartingPosition();
			myRobot.Drive(0.40,0.0);
		}
		return false;
	}

    bool AutonomousDislodgeCollectorBack(bool reset = false)
    {
        if (reset == true)
        {
            autonDislodge.Reset();
            autonDislodge.Start();
        }
        
        if (AutonomousCollectBackFast(999.0,0.4,false,reset) == false)
        {
            autonDislodge.Reset();
            autonDislodge.Start();
        }
        else if (autonDislodge.Get() > 0.2)
        {
            return true;
        }
        else
        {
            collector.LeaveStartingPosition();
            collector.Idle();
            myRobot.Drive(0.0, 0.0);
        }
        return false;
    }
    
    bool AutonomousDislodgeAlternate(bool reset = false)
    {
        if (reset == true)
        {
            autonDislodge.Reset();
            autonDislodge.Start();
            robotClimberExtend.Set(true);
            robotClimberRetract.Set(false);
        }
        
        collector.LeaveStartingPosition(true);
        collector.Idle();
        if (autonDislodge.Get() > 0.1)
        {
            robotClimberExtend.Set(false);
            robotClimberRetract.Set(true);
        }
        
        if (collector.GetPosition() == collector.DOWN)
        {
            return true;
        }
        return false;
    }
	    
	/**
	 * Note: the angle from the gyro accumulates the directional change positive or negative, and doesn't roll over
	 * the angle provided needs to take that into consideration (i.e. don't say 270 degrees if you want to rotate -90)
	 */
	bool AutonomousTurnToAngle(float turnedAngle, bool reset)
	{
	    static Timer onTargetTime;
		if (reset == true)
		{
			rotationPID.SetSetpoint(turnedAngle);
			rotationPID.Enable();
			
			onTargetTime.Reset();
		}
		
		if (rotationPID.OnTarget())
		{
		    onTargetTime.Start();
		}
		else
		{
		    onTargetTime.Reset();
		}
		
		if (onTargetTime.Get() > 0.5)
		{
			rotationPID.Disable();
			return true;
		}
		return false;
	}

    void AutonThreeFrisbeeCorner(void)
    {
        frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
        switch (autonStepCount)
        {

        case 0:
            frisbeeShooter.SetRpm(RPM_AUTONOMOUS_CORNER_SHOTS);
            if (autonReset)
            {
                autonDrivingForward.Reset();
                autonDrivingForward.Start();
            }
            myRobot.Drive(0.0,0.0);
            if (autonDrivingForward.Get() >=  0.2)
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
            if (AutonomousShoot(6,true,autonReset, 1.5) == true)
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
            robotClimberExtend.Set(true);
            robotClimberRetract.Set(false);
            if (autonReset)
            {
                autonDrivingForward.Reset();
                autonDrivingForward.Start();
            }
            myRobot.Drive(0.0,0.0);
            if (autonDrivingForward.Get() >=  0.2)
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
            collector.LeaveStartingPosition();

            robotClimberExtend.Set(false);
            robotClimberRetract.Set(true);
            
            if (AutonomousLowerCollector() == true)
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
            driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
            timeInAutonomous.Stop();
            autonReset = true;
            autonStepCount++;
            break;
        case 5:
            autonReset = true;
            break;
        }
    }

    void AutonThreeFrisbeeCenter(void)
    {
        bool condition1, condition2;
        frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
        switch (autonStepCount)
        {

        case 0:
            frisbeeShooter.SetRpm(RPM_AUTONOMOUS_FIRST_SHOTS);
            if (autonReset)
            {
                autonDrivingForward.Reset();
                autonDrivingForward.Start();
            }
            myRobot.Drive(0.0,0.0);
            if (autonDrivingForward.Get() >=  0.2)
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
            if (AutonomousShoot(6,true,autonReset, 1.5) == true)
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
        
    void AutonSevenFrisbeeForward(void)
        {
            bool condition1, condition2;
            frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
            switch (autonStepCount)
            {
            case 0:
                frisbeeShooter.SetRpm(RPM_AUTONOMOUS_FIRST_SHOTS);
                if (autonReset)
                {
                    autonDrivingForward.Reset();
                    autonDrivingForward.Start();
                }
                myRobot.Drive(0.0,0.0);
                if (autonDrivingForward.Get() >=  0.2)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 1:     //Shoot 3 frisbees (3sec) and lower collector

                condition2 = AutonomousShoot(3,true,autonReset);
                //condition2 = AutonomousShoot(3,true,autonReset,0.0, RPM_AUTONOMOUS_LAST_SHOT);
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

                //frisbeeShooter.SetRpm(RPM_AUTONOMOUS_FIRST_SHOTS);
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
            case 3:         //Drive forward and collect

                frisbeeShooter.SetRpm(RPM_AUTONOMOUS_LAST_FOUR);
                //frisbeeShooter.SetRpm(0);
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
            case 4: //Load frisbees
                myRobot.Drive(0.30 + autonSpeedCorrect,(-autonTurnAmount));
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
            case 5: //Drive forward and lower collector
                if (AutonomousCollectForward(999.0,0.75,true,autonReset,false) == true)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 6: //Stop motor and begin loading
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

                condition1 = AutonomousCollectBackFast(999.0,SEVEN_SHOT_BACKUP_TIME,false,autonReset);
                condition2 = AutonomousLoadFrisbees(true,false, 2.1,0.0,0.35);
                if (condition2)
                {
                    frisbeeShooter.SetRpm(RPM_AUTONOMOUS_LAST_FOUR);
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
                if (autonReset)
                {
                    autonDrivingForward.Reset();
                    autonDrivingForward.Start();
                }
                myRobot.Drive(0.0,0.0);
                if (autonDrivingForward.Get() >=  0.5)
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
                if (AutonomousShoot(4,false,autonReset,0.0,RPM_AUTONOMOUS_LAST_SHOT))
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
                driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
                timeInAutonomous.Stop();
                autonReset = true;
                autonStepCount++;
                break;
            case 11:
                //Shoot 4 frisbees (4 seconds)
                if (AutonomousShoot(4,false,autonReset, 0.6))
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
                autonReset = true;
                break;
                    
            }
                    
        }


};

START_ROBOT_CLASS(RobotDemo);

