#include "WPILib.h"
#include "Shooter.h"
#include "TargetCamera.h"
#include "Collector.h"
#include "Donuts"
#include "customPIDs.h"
//#include "math.h"

//#define PID_DEBUG_MODE
//#define PRACTICE_BOT
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
#define TIME_AUTONOMOUS_DISLODGE 0.3
#define TIME_AUTONOMOUS_SPIN_UP 0.5
#define POWER_AUTONOMOUS_SHOTS 0.56
#define RPM_AUTONOMOUS_SHOTS 3850
#define RPM_AUTONOMOUS_CORNER_SHOTS 3600
#define RPM_TELEOP_SHOTS 4000
#define RPM_LAST_TELEOP_SHOT 3950
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
    PWM_RIGHT_DRIVE,
    PWM_LEFT_DRIVE,
    PWM_SHOOTER_WHEEL,
    PWM_BACK_COLLECTOR,
    PWM_FRONT_COLLECTOR,
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
	AUTONOMOUS_MODE_THREE_FRISBEE_CORNER,
	AUTONOMOUS_MODE_FIVE_FRISBEE_FORWARD,
	AUTONOMOUS_MODE_FIVE_FRISBEE_BACK,
	AUTONOMOUS_MODE_SEVEN_FRISBEE_FORWARD,
	AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK,
	AUTONOMOUS_MODE_NINE_FRISBEE,
	AUTONOMOUS_MODE_FEED_FRISBEE,
	AUTONOMOUS_MODE_SIT_AND_SHOOT,
	AUTONOMOUS_MODE_FIVE_SHOT_CORNER,
	AUTONOMOUS_MODE_TESTING
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
	Encoder rightEncoder;
	Encoder leftEncoder;
	GyroControlledTurning rotationControl;
	PIDController rotationPID;
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
//        autonomousMode = AUTONOMOUS_MODE_NINE_FRISBEE;
		compressor.Start();
		rotationPID.Disable();
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
		
        rotationPID.SetInputRange(ROTATION_PID_MIN_INPUT,ROTATION_PID_MAX_INPUT);
        rotationPID.SetOutputRange(ROTATION_PID_MIN_OUTPUT,ROTATION_PID_MAX_OUTPUT);
        rotationPID.SetAbsoluteTolerance(ROTATION_PID_TOLERENCE);
        rotationPID.Disable();
	}

	
	void TeleopPeriodic (void)
	{
#ifdef PID_DEBUG_MODE
        driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "angle: %f", gyro.GetAngle());   
	    if (driverStation->GetDigitalIn(1))
	    {
            static float angle = 0.0;
            static float p = 0.0;
            static float i = 0.0;
            static float d = 0.0;
            static float tol = 0.0;
            static float offset = 0.0;
            static bool reset = true;
            
            collectorServo.SetAngle(operatorStick.GetThrottle());
                
            if (operatorStick.GetTrigger())
            {
                if (reset)
                {
                    gyro.Reset();
                    rotationPID.SetPID(p,i,d);
                    rotationPID.SetAbsoluteTolerance(tol);
                    rotationControl.SetOffset(offset);
                    rotationPID.SetSetpoint(angle);
                    rotationPID.Enable();
                }
                //AutonomousTurnToAngle(angle,reset);
                reset = false;
            }
            else
            {
                
                rotationPID.Disable();
                
                angle = 90.0f;
                p = driverStation->GetAnalogIn(1)/5.0;
                i = driverStation->GetAnalogIn(2)/5.0;
                d = driverStation->GetAnalogIn(3)/5.0;
                tol = driverStation->GetAnalogIn(4);
                offset = ((1-operatorStick.GetThrottle())/2);
                
                reset = true;
        
                driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "p: %f", p);
                driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "i: %f", i);
                driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "d: %f", d);   
                driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "tol: %f", tol);
                driverStationLCD->PrintfLine((DriverStationLCD::Line) 5, "spd offset: %f", offset);
            }
            driverStationLCD->UpdateLCD();
            return;
	    }
#endif //PID_DEBUG_MODE
		static bool stinger_toggled = false;
		static bool climber_toggled = false;
	    compressor.Start();
		//targetCamera.SetDebugMode(operatorStick.GetRawButton(9));
	    float rpm = frisbeeShooter.GetRpm();
		static bool previous_shoot_state = false;
		static int shot_count = 0;
		static Timer shot_timer;
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
		else if ((operatorStick.GetRawButton(3)==true) || (leftStick.GetRawButton(7) == true))
		{
			//barf frisbees
			collector.Feed();
		}
		else if ((operatorStick.GetRawButton(5) == true) || (leftStick.GetRawButton(6) == true))
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
//		
//		autonTurnAmount = gyro.GetAngle() / 100;
//		if (autonTurnAmount > 0.1) autonTurnAmount = 0.1;
//		if (autonTurnAmount < -0.1) autonTurnAmount = -0.1;
//		
//
//		autonSpeedCorrect = (autonTurnAmount) * AUTON_SPEED_CORRECT_FACTOR;
//		if (autonSpeedCorrect < 0.0) autonSpeedCorrect = autonSpeedCorrect * -1.0;
//		
//		if (operatorStick.GetRawButton(7) == true)
//		{
//			myRobot.Drive(0.25 + autonSpeedCorrect, -autonTurnAmount);
//			
//		}
//		else if (operatorStick.GetRawButton(8) == true)
//		{
//			myRobot.Drive(-(0.25 + autonSpeedCorrect) - (autonDrivingBack.Get() * 2.0), autonTurnAmount);//Set speed to 50% and collect
//		}
//		else
//		{
		//frisbeeShooter.ControlSpeed();
		myRobot.TankDrive(-leftStick.GetY(), -rightStick.GetY()); // drive with arcade style (use right stick)
//		}
//		driverStationLCD->PrintfLine((DriverStationLCD::Line) 6, "angle %f", autonTurnAmount);			

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
				autonomousMode = AUTONOMOUS_MODE_THREE_FRISBEE_CORNER;
				break;
			case AUTONOMOUS_MODE_THREE_FRISBEE_CORNER:
				autonomousMode = AUTONOMOUS_MODE_NINE_FRISBEE;
				break;
			case AUTONOMOUS_MODE_NINE_FRISBEE:
//				autonomousMode = AUTONOMOUS_MODE_FIVE_SHOT_CORNER;
//				break;
//			case AUTONOMOUS_MODE_FIVE_SHOT_CORNER:
                autonomousMode = AUTONOMOUS_MODE_TESTING;
			    break;
			case AUTONOMOUS_MODE_TESTING:
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
		case AUTONOMOUS_MODE_THREE_FRISBEE_CORNER:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Corner 3");
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
		case AUTONOMOUS_MODE_NINE_FRISBEE:
			driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "SHOOT ... 9!");
			break;
        case AUTONOMOUS_MODE_FIVE_SHOT_CORNER:
            driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Corner 5!");
            break;
        case AUTONOMOUS_MODE_TESTING:
            driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Testing routine");
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
		case AUTONOMOUS_MODE_THREE_FRISBEE_CORNER:
			AutonThreeFrisbeeCorner();
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
		case AUTONOMOUS_MODE_NINE_FRISBEE:
			//AutonNineFrisbee();
			AutonNineFrisbee();
			break;
		case AUTONOMOUS_MODE_SEVEN_FRISBEE_BACK:
			//AutonSevenFrisbeeBack();
			break;

        case AUTONOMOUS_MODE_FIVE_SHOT_CORNER:
            AutonFiveShotCorner();
            break;
        case AUTONOMOUS_MODE_TESTING:
            AutonSevenFrisbeeFast();
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
	
	bool AutonomousShoot(int quantity, bool delay_after, bool reset = false, float delay_time = 0.0)
	{
#ifdef PRACTICE_BOT
	    return true;
#endif
		bool fast_shooting = false;
		if ((autonomousMode == AUTONOMOUS_MODE_NINE_FRISBEE) || 
            (autonomousMode == AUTONOMOUS_MODE_TESTING))
		{
			fast_shooting = true;
		}
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
	
	bool AutonomousLoadFrisbees(bool use_switch = true, bool reset = false, float time = 3.0, float delay = 0.0)
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
		else if (autonLoading.Get() >= delay)
		{
			collector.Load();
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
			myRobot.Drive(0.30,0.0);
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
		
	void AutonFiveFrisbeeForward(void)
	{
		bool condition1, condition2;
		frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
		switch (autonStepCount)
		{

		case 0: 	//Shoot 3 frisbees (3sec) and lower collector
			frisbeeShooter.SetRpm(RPM_AUTONOMOUS_SHOTS);
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
		case 4:	//Load frisbees and drive back
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
		case 5:	//Wait .1 sec

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
		case 6:
			//Shoot 2 frisbees (2 seconds)
			if (AutonomousShoot(6,false,autonReset,0.5))
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
			frisbeeShooter.SetRpm(0);
			collector.Idle();
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
	
	

	void AutonSevenFrisbeeForward(void)
		{
			bool condition1, condition2;
			frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
			switch (autonStepCount)
			{
			case 0:
				frisbeeShooter.SetRpm(RPM_AUTONOMOUS_SHOTS);
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
			case 1: 	//Shoot 3 frisbees (3sec) and lower collector

				//condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset); 
				condition2 = AutonomousShoot(3,true,autonReset);
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
				condition1 = AutonomousCollectBackFast(999.0,2.35,false,autonReset);
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
				if (AutonomousShoot(8,false,autonReset))
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
				autonReset = true;
				break;
					
			}
					
		}
	
	
    
    void AutonFiveShotCorner(void)
    {
        bool condition1, condition2;
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
        case 1:     //Shoot 3 frisbees (3sec) and lower collector

            //condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset); 
            condition2 = AutonomousShoot(3,true,autonReset);
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
        case 3:         //Drive back and stop
            AutonomousLowerCollector();
            if (AutonomousCollectBackReallyFast(999.0,1.5,false,autonReset)== true)
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
            if (AutonomousTurnToAngle(90,autonReset) )
            {
                autonReset = true;
                autonStepCount++;
            }
            else
            {
                autonReset = false;
            }
            break;
        case 5: //Drive forward and lower collector
            if (AutonomousCollectForward(999.0,1.0,true,autonReset) == true)
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
            if (autonLoading.Get() > 1.5)
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
            condition1 = AutonomousCollectBackFast(999.0,1.5,false,autonReset);
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
            if (AutonomousTurnToAngle(0,autonReset) )
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
            if (AutonomousCollectForward(999.0,1.0,true,autonReset) == true)
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
            //Shoot 4 frisbees (4 seconds)
            if (AutonomousShoot(8,false,autonReset))
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

            driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
            timeInAutonomous.Stop();
            autonReset = true;
            autonStepCount++;
            break;
        case 12:
            autonReset = true;
            break;
                
        }
                
    }
	
	void AutonSevenFrisbeeFast(void)
	{
	    switch (autonStepCount)
        {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
            AutonNineFrisbee();
            break;
        case 10:
            driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
            timeInAutonomous.Stop();
            autonReset = true;
            autonStepCount++;
            break;
        default:
        case 11:
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
				frisbeeShooter.SetRpm(RPM_AUTONOMOUS_SHOTS);
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
			case 1: 	//Shoot 3 frisbees (3sec) and lower collector

				//TODO condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset); 
				condition2 = AutonomousShoot(3,true,autonReset);
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
				//not needed, will be down early enough (.65) sec 
				if (condition1)condition2 = collector.Lower();
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
				if (AutonomousCollectForward(999.0,0.7,true,autonReset,false)== true)
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
				myRobot.Drive(0.40 + autonSpeedCorrect,-autonTurnAmount);
				if (AutonomousLoadFrisbees(true,autonReset,1.5, 0.1) )
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
				if (AutonomousCollectForward(999.0,0.55,true,autonReset,true,false) == true)
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
				AutonomousLoadFrisbees(true,autonReset, 1.5 ,0.1);
				if (autonLoading.Get() > 0.1)
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
				if (AutonomousLoadFrisbees(true,false, 1.5, 0.1))
				{
					collector.Lower();
				}
				if (AutonomousCollectBackReallyFast(999.0,2.45,false,autonReset))
				{
					autonReset = true;
					autonStepCount++;
				}
				else
				{
					autonReset = false;
				}
				break;
			case 8:	//stop
				if (autonReset)
				{
					autonDrivingForward.Reset();
					autonDrivingForward.Start();
				}
				myRobot.Drive(0.0,0.0);
				if (autonDrivingForward.Get() >=  0.1)
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
				//Shoot 4
				if (AutonomousShoot(4,false,autonReset))
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
				if (AutonomousCollectBackReallyFast(999.0,1.75,true, autonReset,false))
				{
					autonReset = true;
					autonStepCount++;
				}
				else
				{
					autonReset = false;
				}
				break;
			case 11:	//Stop
                AutonomousCollectForward(999.0,1.6,false,autonReset,true,false);
				if (autonDrivingForward.Get() >=  0.1)
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
				condition1 = AutonomousLoadFrisbees(false, autonReset, 1.4, 0.1);
				condition2 = AutonomousCollectForward(999.0,1.6,false,false,true,false);
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
			case 13:
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
			case 14:
				driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
				timeInAutonomous.Stop();
				autonReset = true;
				autonStepCount++;
				break;
			case 15:
				autonReset = true;
				break;
					
			}
					
		}	
	

	



};

START_ROBOT_CLASS(RobotDemo);

