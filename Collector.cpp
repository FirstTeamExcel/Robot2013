# include "Collector.h"

#define TIME_TO_TRAVEL_UP 0.85
#define TIME_TO_TRAVEL_DOWN 0.60
#define TIME_TO_LEAVE_STARTING_POSITION 0.6
#define TIME_TO_DELAY_TILTER 0.090
#define SPEED_TO_BACKROLL_TILTER 0.75
#define TIME_TO_DROP_START 1.0
#define SPECIAL_BACK_ROLLER_LOADING_SPEED 1.00

#define DISABLE_ACCELEROMETER
#define ACCELEROMETER_SWING_LIMIT 0.75f
#define ACCELEROMETER_LOADING_CORRECTION_FACTOR 0.3f
    
Collector::Collector (int front_collector_channel,
        int back_collector_channel,
        int lifter_extend_channel,
        int lifter_retract_channel,
        int tilter_extend_channel,
        int tilter_retract_channel):
            frontCollector (front_collector_channel),
            backCollector (back_collector_channel),
            lifterExtend (lifter_extend_channel),
            lifterRetract (lifter_retract_channel),
            tilterExtend (tilter_extend_channel),
            tilterRetract (tilter_retract_channel),
            extraLifterExtend(2, 3),
            extraLifterRetract(2, 4),
            accelerometer(1)
{
    position = DOWN;
    loadSpeed=0.38;
    feedSpeed=1.0;
    collectSpeed=0.6;
    timeTraveling.Stop();
    ds = DriverStation::GetInstance();
    
}

void Collector::Feed(bool feed_forward)
{
	float speed = feedSpeed;
	if (feed_forward == true)
	{
		speed = -feedSpeed;
	}
    switch (position)
    {
        case UP:
            frontCollector.Set(speed);
            backCollector.Set(speed);
            break;
        case DOWN:
            frontCollector.Set(speed);
            backCollector.Set(speed);
            break;
        case TRAVELING_UP:
            frontCollector.Set(0);
            backCollector.Set(0);
            if (timeTraveling.Get()>= TIME_TO_TRAVEL_UP)
            {
                position = UP;
            }
            break;
        case TRAVELING_DOWN:
            frontCollector.Set(0);
            backCollector.Set(0);
            if (timeTraveling.Get()>= TIME_TO_TRAVEL_DOWN)
            {
                position = DOWN;
            }
            break;
        case STARTING_POSITION:
            frontCollector.Set(speed);
            backCollector.Set(speed);
            break;
        case LEAVING_STARTING_POSITION:
            if (timeTraveling.Get()>= TIME_TO_LEAVE_STARTING_POSITION)
            {
                position = UP;
            }
            break;
        case LEAVING_START_DROP:
            if (timeTraveling.Get()>= TIME_TO_DROP_START)
            {
                position = DOWN;
            }
            break;
    }
}

void Collector::Load(float temp_load_speed)
{
    float accel_correction;
    float swinging_force;
    float speed_temp;
    switch (position)
       {
       case UP:
#ifdef DISABLE_ACCELEROMETER
           
//           frontCollector.Set(-(ds->GetAnalogIn(1)));
//           backCollector.Set(-(ds->GetAnalogIn(2)));
           if (temp_load_speed != 0.0)
           {
               frontCollector.Set(-temp_load_speed);
           }
           else
           {
               frontCollector.Set(-loadSpeed);
           }
           backCollector.Set(-SPECIAL_BACK_ROLLER_LOADING_SPEED);
#else
           //Determine if the robot is spinning too quickly to load
           swinging_force = accelerometer.GetAcceleration(accelerometer.kAxis_Y);
           if ((swinging_force < ACCELEROMETER_SWING_LIMIT) &&
               (swinging_force > -ACCELEROMETER_SWING_LIMIT))
           {
               //Determine how much forward/backward acceleration the robot has
               accel_correction = accelerometer.GetAcceleration(accelerometer.kAxis_X) * ACCELEROMETER_LOADING_CORRECTION_FACTOR;
               
               //Correct the front roller speed for acceleration, to a minimum of 0.15
               speed_temp = loadSpeed + accel_correction;
               if (speed_temp < 0.15)speed_temp = 0.15;
               frontCollector.Set(-speed_temp);
               
               //Correct the back roller speed for acceleration, to a minimum of 0.4
               speed_temp = SPECIAL_BACK_ROLLER_LOADING_SPEED + accel_correction;
               if (speed_temp < 0.4)speed_temp = 0.4;
               backCollector.Set(-speed_temp);
           }
           else
           {
               frontCollector.Set(0.0);
               backCollector.Set(0.0);
           }
#endif 
           break;
       case DOWN:
       case TRAVELING_DOWN:
           frontCollector.Set(0);
           backCollector.Set(0);
           lifterExtend.Set(true);
           extraLifterExtend.Set(true);
           extraLifterRetract.Set(false);
           lifterRetract.Set(false);
           position = TRAVELING_UP;
           timeTraveling.Reset ();
           timeTraveling.Start ();
           break;
       case TRAVELING_UP:
           TravelingUp();
           break;
       case STARTING_POSITION:
           frontCollector.Set(0);
           backCollector.Set(0);
           break;
       case LEAVING_STARTING_POSITION:
           if (timeTraveling.Get()>= TIME_TO_LEAVE_STARTING_POSITION)
           {
               position = UP;
           }
           break;
       case LEAVING_START_DROP:
           if (timeTraveling.Get()>= TIME_TO_DROP_START)
           {
               position = DOWN;
           }
           break;
       }
}

void Collector::Collect(void)
{
    switch (position)
       {
       case UP:
       case TRAVELING_UP:
           frontCollector.Set(0);
           backCollector.Set(0);
           lifterRetract.Set(true);               
           lifterExtend.Set(false);
           extraLifterExtend.Set(false);
           extraLifterRetract.Set(true);
           tilterExtend.Set(false);
           tilterRetract.Set(true);
           position = TRAVELING_DOWN;
           timeTraveling.Reset ();
           timeTraveling.Start ();
           break;
       case DOWN:
           frontCollector.Set(collectSpeed);
           backCollector.Set(-collectSpeed);
           break;
       case TRAVELING_DOWN:
           frontCollector.Set(0);
           backCollector.Set(0);
           if (timeTraveling.Get()>= TIME_TO_TRAVEL_DOWN)
           {
               position = DOWN;
           }
           break;
       case STARTING_POSITION:
           frontCollector.Set(0);
           backCollector.Set(0);
           break;
       case LEAVING_STARTING_POSITION:
           if (timeTraveling.Get()>=TIME_TO_LEAVE_STARTING_POSITION)
           {
               position = UP;
           }
           break;
       case LEAVING_START_DROP:
           if (timeTraveling.Get()>= TIME_TO_DROP_START)
           {
               position = DOWN;
           }
           break;
       }
}
           

void Collector::Idle(void)
{
    frontCollector.Set(0);
    backCollector.Set(0);
    switch (position)
       {
       case UP:
           
           break;
       case DOWN:
           break;
       case TRAVELING_UP:
           TravelingUp();
           break;
       case TRAVELING_DOWN:
           if (timeTraveling.Get()>= TIME_TO_TRAVEL_DOWN)
           {
               position = DOWN;
           }
           break;
       case STARTING_POSITION:
           break;
       case LEAVING_STARTING_POSITION:
           if (timeTraveling.Get()>=TIME_TO_LEAVE_STARTING_POSITION)
           {
               position = UP;
           }
           break;
       case LEAVING_START_DROP:
           if (timeTraveling.Get()>= TIME_TO_DROP_START)
           {
               position = DOWN;
           }
           break;
       }
}

void Collector::EnterStartingPosition(void)
{
   position = STARTING_POSITION;
   timeTraveling.Reset();
   timeTraveling.Stop();
   lifterExtend.Set(true);
   extraLifterExtend.Set(true);
   lifterRetract.Set(false);
   extraLifterRetract.Set(false);
   tilterExtend.Set(false);
   tilterRetract.Set(true);
}

void Collector::LeaveStartingPosition(bool drop_to_the_floor)
{
	if (position == STARTING_POSITION)
	{
	    timeTraveling.Reset();
	    timeTraveling.Start();
	    if (drop_to_the_floor == true)
	    {
	        position = LEAVING_START_DROP;
	        lifterExtend.Set(false);
	        extraLifterExtend.Set(false);
	        lifterRetract.Set(true);
	        extraLifterRetract.Set(true);
	        tilterExtend.Set(false);
	        tilterRetract.Set(true);
	    }
	    else
	    {
	        position = LEAVING_STARTING_POSITION;
	        lifterExtend.Set(true);
	        extraLifterExtend.Set(true);
	        lifterRetract.Set(false);
	        extraLifterRetract.Set(false);   
	        tilterExtend.Set(true);
	        tilterRetract.Set(false);
	    }
	}
}


bool Collector::Raise(void)
{
    bool retVal = false;
    frontCollector.Set(0);
    backCollector.Set(0);
    switch (position)
    {
    case DOWN:
    case TRAVELING_DOWN:
        timeTraveling.Reset ();
        timeTraveling.Start ();
        position = TRAVELING_UP;
    case TRAVELING_UP:
        lifterRetract.Set(false);               
        lifterExtend.Set(true);
        extraLifterRetract.Set(false);
        extraLifterExtend.Set(true);
        tilterExtend.Set(true);
        tilterRetract.Set(false);
        if (timeTraveling.Get()>= TIME_TO_TRAVEL_UP)
        {
           position = UP;
        }
        break;
    case STARTING_POSITION:
      	   break;
    case LEAVING_STARTING_POSITION:
        if (timeTraveling.Get()>=TIME_TO_LEAVE_STARTING_POSITION)
        {
            position = UP;
        }
        break;
    case LEAVING_START_DROP:
        if (timeTraveling.Get()>= TIME_TO_DROP_START)
        {
            position = DOWN;
        }
        break;
    case UP:
        retVal = true;
        break;
    }
    	
	
    return retVal;
}

void Collector::TravelingUp(void)
{
    if (backCollector.Get() == SPEED_TO_BACKROLL_TILTER)
    {
        backCollector.Set(0.21);
    }
    else
    {
        backCollector.Set(0);
    }
    frontCollector.Set(0);
    if (timeTraveling.Get() >= TIME_TO_DELAY_TILTER)
    {
        if (tilterExtend.Get() == false)
        {
            backCollector.Set(SPEED_TO_BACKROLL_TILTER);
        }
        tilterExtend.Set(true);
        tilterRetract.Set(false);
    }
    if (timeTraveling.Get()>= TIME_TO_TRAVEL_UP)
    {
        position = UP;
    }
}
bool Collector::Lower(void)
{
    bool retVal = false;
	frontCollector.Set(0);
   	backCollector.Set(0);
    switch (position)
    {
    case UP:
    case TRAVELING_UP:
        timeTraveling.Reset ();
        timeTraveling.Start ();
        position = TRAVELING_DOWN;
    case TRAVELING_DOWN:
        lifterRetract.Set(true);               
        lifterExtend.Set(false);
        extraLifterRetract.Set(true);
        extraLifterExtend.Set(false);
        tilterExtend.Set(false);
        tilterRetract.Set(true);
        if (timeTraveling.Get()>= TIME_TO_TRAVEL_DOWN)
        {
            position = DOWN;
        }
        break;
    case STARTING_POSITION:
        
        break;
    case LEAVING_STARTING_POSITION:
        if (timeTraveling.Get()>=TIME_TO_LEAVE_STARTING_POSITION)
        {
            position = UP;
        }
        break;
    case LEAVING_START_DROP:
        if (timeTraveling.Get()>= TIME_TO_DROP_START)
        {
            position = DOWN;
        }
        break;
    case DOWN:
        retVal = true;
        break;
    }

    return retVal;
}
//float Collector::GetCollectSpeed(void)
//{
//    return collectSpeed; 
//}
//
//void Collector::SetCollectSpeed(float speed)
//{
//    collectSpeed = speed;
//}
//
//float Collector::GetFeedSpeed(void)
//{
//    return feedSpeed;
//}
//
//void Collector::SetFeedSpeed(float speed)
//{
//    feedSpeed = speed; 
//}
//
//float Collector::GetLoadSpeed(void)
//{
//    return loadSpeed;
//}
//
//void Collector::SetLoadSpeed(float speed)
//{
//    loadSpeed = speed;
//}

