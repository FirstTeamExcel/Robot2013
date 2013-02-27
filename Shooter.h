#include "Wpilib.h"

#ifndef SHOOTER_H_
#define SHOOTER_H_
#define FPGA_TIME_TO_MINUTES_FACTOR (60*1000*1000)
class Shooter
{
private:  //shooter attributes
	int target_rpm;
	Counter *wheel_counter;
	DigitalInput *wheel_trigger;
	Talon wheel_motor;
	float max_power;
	float ramp_up_rate;
	int counts_per_revolution;
	float ramp_down_rate;
	float filter_constant;
	unsigned long int last_timestamp;
	float previous_rpm;
	float GetRpm (void);
	//void WheelTriggerInterrupt (uint32_t interruptAssertedMask, void *param);
	
public: //shooter functions
	Shooter (int motor_channel, DigitalInput *trigger);
	Shooter (int motor_channel, Counter *counter);
	bool ControlSpeed (void);
	void SetPower (float power_level);
	void SetRpm (int rpm);
	bool IsReady();
	void SetMaxPower (float power);
	void SetRampUpRate (float rate);
	void SetCountsPerRevolution (int counts);
	void SetRampDownRate (float rate);
	void SetFilterConstant (float filter_value);
	float GetPreviousRpm (void){return previous_rpm;}
};






#endif /*SHOOTER_H_*/
