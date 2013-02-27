#include "Shooter.h"
void WheelTriggerInterrupt (uint32_t interruptAssertedMask, void *param);

DriverStationLCD *driverStationLCD;
Shooter::Shooter (int motor_channel, DigitalInput *trigger):
	wheel_motor (motor_channel)
{
     //wheel_counter = (void*)0;
     wheel_trigger = trigger;
//     wheel_trigger->EnableInterrupts();
//     wheel_trigger->SetUpSourceEdge(true, false);
//     wheel_trigger->RequestInterrupts(WheelTriggerInterrupt, (void *)&previous_rpm);
     max_power=1.0f;
     ramp_up_rate=0.2;
     counts_per_revolution=400;
     ramp_down_rate=1.0;
     filter_constant=0.0;
    driverStationLCD = DriverStationLCD::GetInstance();
    last_timestamp = 0;
}
Shooter::Shooter (int motor_channel, Counter *counter):
	wheel_motor (motor_channel)
{
	 wheel_counter = counter;
	 //wheel_trigger = (void*)0;
	 max_power=1.0f;
	 ramp_up_rate=0.2;
     counts_per_revolution=400;
	 ramp_down_rate=1.0;
	 filter_constant=0.0;
	 last_timestamp = 0;
    driverStationLCD = DriverStationLCD::GetInstance();
}
bool Shooter::ControlSpeed (void)
{
	bool is_ready = false;
	if (GetRpm () < target_rpm)
	{
		if ((wheel_motor.Get ()+ ramp_up_rate)< max_power)
		{
			wheel_motor.Set (wheel_motor.Get() + ramp_up_rate);
		}
		else 
		{
			wheel_motor.Set (max_power);
		}
	}
	else
	{
		wheel_motor.Set (0);
		is_ready = true;
	}
	return is_ready;
}
void Shooter::SetPower (float power_level)
{
	wheel_motor.Set (power_level);
	
}
void Shooter::SetRpm (int rpm)
{
	target_rpm = rpm;
	
}
bool Shooter::IsReady()
{
return false;	
}
void Shooter::SetMaxPower (float power)
{
	max_power = power;
}
void Shooter::SetRampUpRate (float rate)
{
	ramp_up_rate = rate;
}
void Shooter::SetCountsPerRevolution (int counts)
{
	counts_per_revolution = counts;
}
void Shooter::SetRampDownRate (float rate)
{
	ramp_down_rate = rate;
}
void Shooter::SetFilterConstant (float filter_value)
{
	filter_constant = filter_value;
}
float Shooter::GetRpm(void)
{
    if (wheel_counter != (void*)0)
    {
        if (last_timestamp == 0)
        {
            last_timestamp = GetFPGATime();
            wheel_counter->Start ();
            driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "initialized rpm");
            return 0;
        }
        unsigned long int current_time = GetFPGATime();
        INT32 count_value = wheel_counter->Get();
        unsigned long int time_difference = current_time - last_timestamp;

        driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "time difference: %d", time_difference);
        driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "counts: %d", count_value);
        float revolutions = (float)count_value/(float)counts_per_revolution;
        previous_rpm = revolutions/(((float)time_difference)/((float)FPGA_TIME_TO_MINUTES_FACTOR));
        last_timestamp = GetFPGATime();
        wheel_counter->Reset();  
        
        driverStationLCD->UpdateLCD();
    }  	
    return previous_rpm;
}
void WheelTriggerInterrupt (uint32_t interruptAssertedMask, void *param)
{
//    float *rpm = (float *)param;
//    *rpm = 2;
}



