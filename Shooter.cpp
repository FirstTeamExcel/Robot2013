#include "Shooter.h"

#define POWER_DURING_SHOT 0.50
#define TAKE_BACK_TIME 0.4
#define SPEED_TOLERANCE 0.01

Shooter::Shooter (int motor_channel, 
		Counter *counter,
		int numanumamatic_extend_channel,
		int numanumamatic_retract_channel):
	wheel_motor (motor_channel),
	numanumamaticExtend (numanumamatic_extend_channel),
	numanumamaticRetract (numanumamatic_retract_channel),
	shooterSemaphore()
{
	 numanumamaticExtend.Set(false);
	 numanumamaticRetract.Set(true);
	 wheel_counter = counter;
	 max_power=1.0f;
	 ramp_up_rate=1.0;
     //counts_per_revolution=1;
	 ramp_down_rate=1.0;
	 //filter_constant=0.0;
	 last_timestamp = 0;
	 timeTraveling.Reset();
	 timeTraveling.Start();
	 state = READY;
	 speedControl = false;
	 if (wheel_counter != (void *)0)
	 {
		 wheel_counter->Reset();
		 wheel_counter->SetUpSourceEdge(true,false);
		 wheel_counter->SetMaxPeriod(2.0);
		 wheel_counter->Start();
	 }
}

void Shooter::SetPower (float power_level)
{
	shooterSemaphore.take();
	wheel_motor.Set (power_level);
	speedControl = false;
	Pause();
	shooterSemaphore.give();
}
void Shooter::SetRpm (float rpm)
{	
	shooterSemaphore.take();
	speedControl = true;
	if (rpm == 0)
	{
		target_sec_per_revolution = 0;
		target_sec_per_revolution_slowdown = 0;
	}
	else
	{
		target_sec_per_revolution = ((double)60.0) / ((double)rpm);
		target_sec_per_revolution_slowdown = target_sec_per_revolution * (1.0 + SPEED_TOLERANCE);
		target_sec_per_revolution_overshoot = target_sec_per_revolution * (1.0 - SPEED_TOLERANCE);
	}
	
	//if the speed control task isn't running, start it.
	if ((TaskRunning() == false) && (state == READY))
	{
		last_motor_command = 0;
		Start();	//Begins the speed control task, only runs if speedControl ==true
	}
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
	else if (speedControl && upToSpeed)
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
//void Shooter::SetCountsPerRevolution (int counts)
//{
//	shooterSemaphore.take();
//	counts_per_revolution = counts;
//	shooterSemaphore.give();
//}
void Shooter::SetRampDownRate (float rate)
{
	shooterSemaphore.take();
	ramp_down_rate = rate;
	shooterSemaphore.give();
}
//void Shooter::SetFilterConstant (float filter_value)
//{
//	shooterSemaphore.take();
//	filter_constant = filter_value;
//	shooterSemaphore.give();
//}

void Shooter::Run(void)
{
	if (wheel_counter == (void *)0)
		return;
	shooterSemaphore.take();
	//If a revolution period is longer than the target, drive the motor
	double period = wheel_counter->GetPeriod();
	float motor_command = 0.0;
	
	
	if (target_sec_per_revolution == 0)
	{
		upToSpeed = false;
	}
	else if (target_sec_per_revolution < period)
	{
//		if ((last_motor_command + ramp_up_rate) < max_power)
//		{
//			wheel_motor.Set (wheel_motor.Get() + ramp_up_rate);
//			motor_command = last_motor_command + ramp_up_rate;
//		}
//		else 
//		{
//			wheel_motor.Set (max_power);
//			motor_command = max_power;
//		}
		motor_command = max_power;
		//if target * 1.125 < period then we've slowed down enough to not be up to speed
		
	}
	else
	{
		upToSpeed = true;
	}
	if ((target_sec_per_revolution_slowdown < period) || (target_sec_per_revolution_overshoot > period)) 
	{
		upToSpeed = false;
	}
	
	if (last_motor_command != motor_command)
		wheel_motor.Set(motor_command);
	
	last_motor_command = motor_command;
	
	shooterSemaphore.give();
}

float Shooter::GetRpm(void)
{
	float retVal = -2.0;
	if (wheel_counter == (void *)0)
		return -1.0;
	
	shooterSemaphore.take();
	if (wheel_counter->GetStopped())
	{
		shooterSemaphore.give();
		return 0.0;
	}
	
	retVal = (float)wheel_counter->GetPeriod();
	shooterSemaphore.give();
	
	retVal = (60 / retVal); 
    return retVal;
}

bool Shooter::ShootFrisbee (bool fire, bool really_fast)
{
	bool retValue = false;
	float travel_time = 0.5;
	static Timer take_back_timer;
	if (speedControl == true)
	{
		travel_time = 0.3;
		if (really_fast == true)travel_time = 0.2;
	}
	
	switch (state)
	{
		case READY:
			if (fire && IsReady () && (timeTraveling.Get() >= travel_time))
			{
				shooterSemaphore.take();
				wheel_motor.Set(POWER_DURING_SHOT);
				last_motor_command = POWER_DURING_SHOT;
				shooterSemaphore.give();
                take_back_timer.Reset();
                take_back_timer.Start();
                upToSpeed = false;
				numanumamaticExtend.Set(true);
				numanumamaticRetract.Set(false);
				timeTraveling.Reset();
				timeTraveling.Start();
				state = FIRED;
				retValue = true;
			}
			break;
		case FIRED:
			if (timeTraveling.Get() >= travel_time)
			{
				timeTraveling.Reset();
				timeTraveling.Start();

				numanumamaticExtend.Set(false);
				numanumamaticRetract.Set(true);
				state = READY;
			}
			break;
			
	}
	if (take_back_timer.Get() >= TAKE_BACK_TIME)
	{
        shooterSemaphore.take();
        Start();
        shooterSemaphore.give();
        
	    take_back_timer.Stop();
	    take_back_timer.Reset();
	}
	return retValue;
}


bool Shooter::DeliberatelySlowPowerBasedFrisbeeShootingTest (void)
{
    static bool shot_fired = true;
    static Timer time;
    
    bool ret_value = false;
    time.Start();
    wheel_motor.Set(0.5);
    if (shot_fired)
    {
        if (time.Get() >= 6.0)
        {
            time.Reset();
            numanumamaticExtend.Set(false);
            numanumamaticRetract.Set(true);
            shot_fired = false;
        }
    }
    else
    {
        if (time.Get() >= 6.0)
        {
            time.Reset();
            numanumamaticExtend.Set(true);
            numanumamaticRetract.Set(false);
            shot_fired = true;
            ret_value = true;
        } 
    }
    
    return ret_value;
}

