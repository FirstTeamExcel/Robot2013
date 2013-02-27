#include "Shooter.h"

Shooter::Shooter (int motor_channel, 
		Counter *counter,
		int numanumamatic_extend_channel,
		int numanumamatic_retract_channel):
	wheel_motor (motor_channel),
	numanumamaticExtend (numanumamatic_extend_channel),
	numanumamaticRetract (numanumamatic_retract_channel),
	shooterSemaphore()
{
	 wheel_counter = counter;
	 //wheel_trigger = (void*)0;
	 max_power=1.f;
	 ramp_up_rate=0.2;
     counts_per_revolution=400;
	 ramp_down_rate=1.0;
	 filter_constant=0.0;
	 last_timestamp = 0;
	 timeTraveling.Reset();
	 timeTraveling.Start();
	 state = READY;
	 speedControl = false;
}

void Shooter::SetPower (float power_level)
{
	wheel_motor.Set (power_level);

	shooterSemaphore.take();
	speedControl = false;
	Pause();
	shooterSemaphore.give();

}
void Shooter::SetRpm (int rpm)
{	
	shooterSemaphore.take();
	speedControl = true;
	target_rpm = rpm;
	Start();	//Begins the speed control task, only runs if speedControl ==true
	shooterSemaphore.give();

}
bool Shooter::IsReady()
{
	//TODO initialize to false.
	bool is_ready = false;
	shooterSemaphore.take();
	if (speedControl == false)
	{
		if (wheel_motor.Get() != 0)
		{
			is_ready = true;
		}
	}
	else if (speedControl && (previous_rpm > (0.95 *target_rpm)))
	{
		is_ready = true;
	}
	shooterSemaphore.give();
		
	return is_ready;	
}
void Shooter::SetMaxPower (float power)
{
	shooterSemaphore.take();
	max_power = power;
	shooterSemaphore.give();
			
}
void Shooter::SetRampUpRate (float rate)
{
	shooterSemaphore.take();
	ramp_up_rate = rate;
	shooterSemaphore.give();
}
void Shooter::SetCountsPerRevolution (int counts)
{
	shooterSemaphore.take();
	counts_per_revolution = counts;
	shooterSemaphore.give();
}
void Shooter::SetRampDownRate (float rate)
{
	shooterSemaphore.take();
	ramp_down_rate = rate;
	shooterSemaphore.give();
}
void Shooter::SetFilterConstant (float filter_value)
{
	shooterSemaphore.take();
	filter_constant = filter_value;
	shooterSemaphore.give();
}

void Shooter::Run(void)
{
	shooterSemaphore.take();
    if ((wheel_counter != (void*)0) && 
		(speedControl == true) &&	//if using an encoder and we last commanded rpm (rather than power) 	
		(wheel_counter->Get() > 20)) //Ensure a minimum number of counts to prevent poor resolution
    {
    	
        if (last_timestamp == 0)
        {
            last_timestamp = GetFPGATime();
            wheel_counter->Start ();
            return;
        }
        unsigned long int current_time = GetFPGATime();
        INT32 count_value = wheel_counter->Get();
        unsigned long int time_difference = current_time - last_timestamp;
        float revolutions = (float)count_value;
        if (counts_per_revolution == 1)        	
        {
	        revolutions /= (float)counts_per_revolution;
        	
        }
        //previous_rpm = revolutions/(((float)time_difference)/((float)FPGA_TIME_TO_MINUTES_FACTOR));
        previous_rpm = (revolutions/(float)time_difference) * (float)FPGA_TIME_TO_MINUTES_FACTOR;
        last_timestamp = GetFPGATime();
        wheel_counter->Reset();  

        if (previous_rpm < target_rpm)
    	{
    		if ((wheel_motor.Get () + ramp_up_rate) < max_power)
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
    	}
    }	
	shooterSemaphore.give();
}

float Shooter::GetRpm(void)
{
	float retVal;
	shooterSemaphore.take();
	retVal = previous_rpm;
	shooterSemaphore.give();
    return retVal;
}

bool Shooter::ShootFrisbee (bool fire)
{
	bool retValue = false;
	switch (state)
	{
		case READY:
			if (fire && IsReady () && timeTraveling.Get()>=.5)
			{
				numanumamaticExtend.Set(true);
				numanumamaticRetract.Set(false);
				timeTraveling.Reset();
				timeTraveling.Start();
				state = FIRED;
				retValue = true;
			}
			break;
		case FIRED:
			if (timeTraveling.Get()>=.5)
			{
				timeTraveling.Reset();
				timeTraveling.Start();
			
				numanumamaticExtend.Set(false);
				numanumamaticRetract.Set(true);
				state = READY;
			}
			break;
			
	}
	return retValue;
}






