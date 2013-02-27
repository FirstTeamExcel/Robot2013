# include "Collector.h"


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
            tilterRetract (tilter_retract_channel)
{
    position = DOWN;
    loadSpeed=0.5;
    feedSpeed=0.9;
    collectSpeed=0.5;
    timeTraveling.Stop();
}
void Collector::Feed(void)
{
    switch (position)
    {
        case UP:
            frontCollector.Set(feedSpeed);
            backCollector.Set(feedSpeed);
            break;
        case DOWN:
            frontCollector.Set(feedSpeed);
            backCollector.Set(feedSpeed);
            break;
        case TRAVELING_UP:
            frontCollector.Set(0);
            backCollector.Set(0);
            if (timeTraveling.Get()>= .75)
            {
                position = UP;
            }
            break;
        case TRAVELING_DOWN:
            frontCollector.Set(0);
            backCollector.Set(0);
            if (timeTraveling.Get()>= .5)
            {
                position = DOWN;
            }
            break;
        case STARTING_POSITION:
            frontCollector.Set(-feedSpeed);
            backCollector.Set(feedSpeed);
            break;
        case LEAVING_STARTING_POSITION:
            if (timeTraveling.Get()>=.5)
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
           backCollector.Set(-0.75);
           break;
       case DOWN:
       case TRAVELING_DOWN:
           frontCollector.Set(0);
           backCollector.Set(0);
           lifterExtend.Set(true);
           lifterRetract.Set(false);
           position = TRAVELING_UP;
           timeTraveling.Reset ();
           timeTraveling.Start ();
           break;
       case TRAVELING_UP:
           frontCollector.Set(0);
           backCollector.Set(0);
           if (timeTraveling.Get() >= .15)
           {
        	   if (tilterExtend.Get() == false)
        	   {
        		   backCollector.Set(.5);
        	   }
               tilterExtend.Set(true);
               tilterRetract.Set(false);
           }
           if (timeTraveling.Get()>= .75)
           {
               position = UP;
           }
           break;
       case STARTING_POSITION:
           frontCollector.Set(0);
           backCollector.Set(0);
           break;
       case LEAVING_STARTING_POSITION:
           if (timeTraveling.Get()>=.5)
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
           if (timeTraveling.Get()>= .5)
           {
               position = DOWN;
           }
           break;
       case STARTING_POSITION:
           frontCollector.Set(0);
           backCollector.Set(0);
           break;
       case LEAVING_STARTING_POSITION:
           if (timeTraveling.Get()>=.5)
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
           if (timeTraveling.Get() >= .15)
           {
        	   if (tilterExtend.Get() == false)
        	   {
        		   backCollector.Set(.5);
        	   }
               tilterExtend.Set(true);
               tilterRetract.Set(false);
           }
           if (timeTraveling.Get()>= .75)
           {
               position = UP;
           }
           break;
       case TRAVELING_DOWN:
           if (timeTraveling.Get()>= .5)
           {
               position = DOWN;
           }
           break;
       case STARTING_POSITION:
           break;
       case LEAVING_STARTING_POSITION:
           if (timeTraveling.Get()>=.2)
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
}

void Collector::LeaveStartingPosition(void)
{
    position = LEAVING_STARTING_POSITION;
    timeTraveling.Reset();
    timeTraveling.Start();
    lifterExtend.Set(true);
    lifterRetract.Set(false);
    tilterExtend.Set(true);
    tilterRetract.Set(false);
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

