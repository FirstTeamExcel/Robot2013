
#if 0
    void AutonSevenFrisbeeForward(void)
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

                condition2 = AutonomousShoot(3,true,autonReset);
                //condition2 = AutonomousShoot(3,true,autonReset,0.0, RPM_AUTONOMOUS_LAST_SHOT);
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

                //frisbeeShooter.SetRpm(RPM_AUTONOMOUS_FIRST_SHOTS);
                condition1 = AutonomousDislodgeCollector(TIME_AUTONOMOUS_DISLODGE, autonReset); 
                condition2 = AutonomousLowerCollector();
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
                frisbeeShooter.SetRpm(RPM_AUTONOMOUS_LAST_FOUR);
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
                myRobot.Drive(0.20 + autonSpeedCorrect,(-autonTurnAmount));
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
                if (AutonomousCollectForward(999.0,0.80,true,autonReset) == true)
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
                condition1 = AutonomousCollectBackFast(999.0,2.5,false,autonReset);
                condition2 = AutonomousLoadFrisbees(true,false, 2.0,0.0,0.35);
                if (condition2)
                {
                    collector.Lower();
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
            case 8:
                if (autonReset)
                {
                    autonDrivingForward.Reset();
                    autonDrivingForward.Start();
                }
                myRobot.Drive(0.0,0.0);
                if (autonDrivingForward.Get() >=  0.5)
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
                //Shoot 4 frisbees (4 seconds)
                if (AutonomousShoot(4,false,autonReset,0.0,RPM_AUTONOMOUS_LAST_SHOT))
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
                driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Time: %f", timeInAutonomous.Get());
                timeInAutonomous.Stop();
                autonReset = true;
                autonStepCount++;
                break;
            case 11:
                //Shoot 4 frisbees (4 seconds)
                if (AutonomousShoot(4,false,autonReset, 0.6))
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
                autonReset = true;
                break;
                    
            }
                    
        }
#endif
