# include "Collector.h"


Collector::Collector (int front_collector_channel,
        int back_collector_channel,
        int lifter_left_extend_channel,
        int lifter_left_retract_channel,
        int lifter_right_extend_channel,
        int lifter_right_retract_channel,
        int tilter_extend_channel,
        int tilter_retract_channel):
            frontCollector (front_collector_channel),
            backCollector (back_collector_channel),
            lifterLeftExtend (lifter_left_extend_channel),
            lifterLeftRetract (lifter_left_retract_channel),
            lifterRightExtend (lifter_right_extend_channel),
            lifterRightRetract (lifter_right_retract_channel),
            tilterExtend (tilter_extend_channel),
            tilterRetract (tilter_retract_channel)
{
    position = STARTING_POSITION;
    loadSpeed=0.2;
    feedSpeed=0.5;
    collectSpeed=0.35;
    timeTraveling.Stop();
    frontCollector.Set(0);
    backCollector.Set(0);
}
void Collector::Feed(void)
{
    switch (position)
    {
        case UP:
            frontCollector.Set(-feedSpeed);
            backCollector.Set(feedSpeed);
            break;
        case DOWN:
            frontCollector.Set(-feedSpeed);
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
            frontCollector.Set(0);
            backCollector.Set(0);
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
           frontCollector.Set(loadSpeed);
           backCollector.Set(-loadSpeed);
           break;
       case DOWN:
       case TRAVELING_DOWN:
           frontCollector.Set(0);
           backCollector.Set(0);
           lifterLeftExtend.Set(true);
           lifterRightExtend.Set(true);
           lifterLeftRetract.Set(false);
           lifterRightRetract.Set(false);
           tilterExtend.Set(true);
           tilterRetract.Set(false);
           position = TRAVELING_UP;
           timeTraveling.Reset ();
           timeTraveling.Start ();
           break;
       case TRAVELING_UP:
           frontCollector.Set(0);
           backCollector.Set(0);
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
           frontCollector.Set(0);
           backCollector.Set(0);
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
           lifterRightRetract.Set(true);
           lifterLeftRetract.Set(true);               
           lifterLeftExtend.Set(false);
           lifterRightExtend.Set(false);
           tilterExtend.Set(false);
           tilterRetract.Set(true);
           position = TRAVELING_DOWN;
           timeTraveling.Reset ();
           timeTraveling.Start ();
           break;
       case DOWN:
           frontCollector.Set(collectSpeed);
           backCollector.Set(collectSpeed);
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
           frontCollector.Set(0);
           backCollector.Set(0);
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
           if (timeTraveling.Get()>=.5)
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
   frontCollector.Set(0);
   backCollector.Set(0);
}

void Collector::LeaveStartingPosition(void)
{
    position = LEAVING_STARTING_POSITION;
    timeTraveling.Reset();
    timeTraveling.Start();
    lifterLeftExtend.Set(true);
    lifterRightExtend.Set(true);
    lifterLeftRetract.Set(false);
    lifterRightRetract.Set(false);
    tilterExtend.Set(true);
    tilterRetract.Set(false);
    frontCollector.Set(0);
    backCollector.Set(0);
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

