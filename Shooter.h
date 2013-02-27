#include "Wpilib.h"
#include "jankyTask.h"

#ifndef SHOOTER_H_
#define SHOOTER_H_
#define FPGA_TIME_TO_MINUTES_FACTOR (60*1000*1000)
class Shooter : public JankyTask
{
public:
	typedef enum {FIRED, READY,} SHOOTER_STATE; 
private:  //shooter attributes
	SHOOTER_STATE state;
	//int target_rpm;
	unsigned long int target_usec_per_revolution;
	Counter *wheel_counter;
	Talon wheel_motor;
	Solenoid numanumamaticExtend;
	Solenoid numanumamaticRetract;
	Timer timeTraveling;
	float max_power;
	float ramp_up_rate;
	//int counts_per_revolution;
	float ramp_down_rate;
	//float filter_constant;
	unsigned long int last_timestamp;
	//float previous_rpm;
	ReentrantSemaphore shooterSemaphore;
	bool speedControl;
	bool upToSpeed;
	
public: //shooter functions
	Shooter (int motor_channel, Counter *counter, int numanumamatic_extend_channel, int numanumamatic_retract_channel);
	//bool ControlSpeed (void);
	void SetPower (float power_level);
	float GetPower (void){return wheel_motor.Get();}
	void SetRpm (unsigned long int rpm);
	bool IsReady();
	void SetMaxPower (float power);
	void SetRampUpRate (float rate);
	//void SetCountsPerRevolution (int counts); counts per rev implemented as 1 only
	void SetRampDownRate (float rate);
	//void SetFilterConstant (float filter_value); filter not implemented
	float GetRpm (void);
	bool ShootFrisbee (bool fire);
	void Run(void);
};






#endif /*SHOOTER_H_*/
