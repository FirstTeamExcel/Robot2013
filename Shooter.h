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
	int target_rpm;
	Counter *wheel_counter;
	DigitalInput *wheel_trigger;
	Talon wheel_motor;
	Solenoid numanumamaticExtend;
	Solenoid numanumamaticRetract;
	Timer timeTraveling;
	float max_power;
	float ramp_up_rate;
	int counts_per_revolution;
	float ramp_down_rate;
	float filter_constant;
	unsigned long int last_timestamp;
	float previous_rpm;
	ReentrantSemaphore shooterSemaphore;
	bool speedControl;
	//void WheelTriggerInterrupt (uint32_t interruptAssertedMask, void *param);
	
public: //shooter functions
	Shooter (int motor_channel, Counter *counter, int numanumamatic_extend_channel, int numanumamatic_retract_channel);
	//bool ControlSpeed (void);
	void SetPower (float power_level);
	float GetPower (void){return wheel_motor.Get();}
	void SetRpm (int rpm);
	bool IsReady();
	void SetMaxPower (float power);
	void SetRampUpRate (float rate);
	void SetCountsPerRevolution (int counts);
	void SetRampDownRate (float rate);
	void SetFilterConstant (float filter_value);
	float GetRpm (void);
	bool ShootFrisbee (bool fire);
	void Run(void);
};






#endif /*SHOOTER_H_*/
