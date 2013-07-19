#if 0
    void AutonNineFrisbee(void)
        {
            bool condition1, condition2;
            frisbeeShooter.ShootFrisbee(false);//Service the shooter to retract the firing piston
            switch (autonStepCount)
            {
            case 0:
                frisbeeShooter.SetRpm(RPM_AUTONOMOUS_FIRST_SHOTS);
                if (autonReset)
                {
                    autonDrivingForward.Reset();
                    autonDrivingForward.Start();
                }
                myRobot.Drive(0.0,0.0);
                if (autonDrivingForward.Get() >=  0.2)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 1:     //Shoot 3 frisbees (3sec) and lower collector

                //TODO condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset); 
                condition2 = AutonomousShoot(3,true,autonReset);
                if (condition2)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 2:
                condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset);
                //not needed, will be down early enough (.65) sec 
                if (condition1)
                {
                    if (collector.GetPosition() == collector.UP)
                    {
                        autonDrivingForward.Reset();
                        autonDrivingForward.Start();   
                    }
                    
                    collector.Lower();
                    
                    if (autonDrivingForward.Get() >=  0.3)
                    {
                        condition2 = true;
                    }
                }
                                
                if (condition1 && condition2)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 3:         //Drive forward and collect
                if (AutonomousCollectForward(999.0,1.1,true,autonReset)== true)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
        
                break;
            case 4: //Load frisbees
                myRobot.Drive(0.20 + autonSpeedCorrect,((-autonTurnAmount)));
                if (AutonomousLoadFrisbees(true,autonReset,2.0) )
                {
                    if (AutonomousLowerCollector())
                    {
                        autonReset = true;
                        autonStepCount++;
                    }
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 5: //Drive forward and lower collector
                if (AutonomousCollectForward(999.0,0.80,true,autonReset,true,false) == true)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 6: //Stop motor and begin loading
                myRobot.Drive(0.0,0.0);
                AutonomousLoadFrisbees(true,autonReset);
                if (autonLoading.Get() > 0.25)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 7:
                //Load and Drive Backward without collecting
                if (AutonomousLoadFrisbees(true,false, 1.5, 0.1))
                {
                    collector.Lower();
                }
                if (AutonomousCollectBackReallyFast(999.0,2.0,false,autonReset))
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 8: //stop
                if (autonReset)
                {
                    autonDrivingForward.Reset();
                    autonDrivingForward.Start();
                }
                myRobot.Drive(0.0,0.0);
                if (autonDrivingForward.Get() >=  0.3)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 9:
                //Shoot 4
                if (AutonomousShoot(4,false,autonReset))
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 10:
                if (AutonomousCollectBackReallyFast(999.0,1.55,true, autonReset,false))
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 11:    //Stop
                AutonomousCollectForwardFast(999.0,1.6,false,autonReset,true,false);
                if (autonDrivingForward.Get() >=  0.1)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 12:
                //TODO if loading is delayed by forward momentum, then add a short myRobot.Drive(0.0,0.0) at a specific time for only a short period
                condition1 = AutonomousLoadFrisbees(false, autonReset, 1.9, 0.1);
                condition2 = AutonomousCollectForwardFast(999.0,1.4,false,false,true,false);
                if (condition1 && condition2)
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 13:
                if (AutonomousShoot(2,false,autonReset))
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 14:
                driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
                timeInAutonomous.Stop();
                autonReset = true;
                autonStepCount++;
                //Fall through
            case 15:

                if (AutonomousShoot(4,false,autonReset))
                {
                    autonReset = true;
                    autonStepCount++;
                }
                else
                {
                    autonReset = false;
                }
                break;
            case 16:
                autonReset = true;
                break;
                    
            }
                    
        }   
#endif
