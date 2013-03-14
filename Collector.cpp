# include "Collector.h"

#define TIME_TO_TRAVEL_UP 1.25
#define TIME_TO_TRAVEL_DOWN 0.75
#define TIME_TO_LEAVE_STARTING_POSITION 1.5
#define TIME_TO_DELAY_TILTER 0.15
#define SPECIAL_BACK_ROLLER_LOADING_SPEED 0.75

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
            extraLifterRetract(2, 4)
{
    position = DOWN;
    loadSpeed=0.45;
    feedSpeed=1.0;
    collectSpeed=0.6;
    timeTraveling.Stop();
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
    }
}

void Collector::Load(void)
{
    switch (position)
       {
       case UP:
           frontCollector.Set(-loadSpeed);
           backCollector.Set(-SPECIAL_BACK_ROLLER_LOADING_SPEED);
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
    	   if (backCollector.Get() == 0.5)
    	   {
    		   backCollector.Set(0.51);
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
        		   backCollector.Set(.5);
        	   }
               tilterExtend.Set(true);
               tilterRetract.Set(false);
           }
           if (timeTraveling.Get()>= TIME_TO_TRAVEL_UP)
           {
               position = UP;
           }
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
           if (timeTraveling.Get() >= TIME_TO_DELAY_TILTER)
           {
        	   if (tilterExtend.Get() == false)
        	   {
        		   backCollector.Set(.5);
        	   }
               tilterExtend.Set(true);
               tilterRetract.Set(false);
           }
           if (timeTraveling.Get()>= TIME_TO_TRAVEL_UP)
           {
               position = UP;
           }
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

void Collector::LeaveStartingPosition(void)
{
	if (position == STARTING_POSITION)
	{
	    position = LEAVING_STARTING_POSITION;
	    timeTraveling.Reset();
	    timeTraveling.Start();
	    lifterExtend.Set(true);
	    extraLifterExtend.Set(true);
	    lifterRetract.Set(false);
	    extraLifterRetract.Set(false);
	    tilterExtend.Set(true);
	    tilterRetract.Set(false);
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
    case UP:
        retVal = true;
        break;
    }
    	
	
    return retVal;
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

