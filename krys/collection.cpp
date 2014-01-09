#include "collection_h.h"
#include "running_h.h"

static char* sDebugPhase = "*****************";

static bool sMoveForwardCompleted = false;
static CollectionState sCollectionState = csMoveBack;
static uint16 sLastMotorCount;
static uint32 sRollerSampleTmr = ROLLER_SAMPLE_TIMER, sCollectTmr = cBallCollectionTimer, sMoveFwdTmr = cWaitForBallDistanceTmr, sWaitTmr = cWaitForBallTimer;
static uint16 sSpeedAdjust = 0;

//pre-cond: on center tape
//post-cond: at back with balls collected
void Collect(uint16 diff)
{
    switch (sCollectionState)
    {
    case csMoveBack: //dbp1
	{
		if (BackSwitchPressed())
        {
			//exit cond
            StopMoving();
            sCollectionState = csCollecting;
            sSpeedAdjust = 0;
        }

        ActivateAcqMotor();

        if (GetRearAveragedSum() > 250)
        {
            sSpeedAdjust = 0;
        }

        PID::QRDUpdate(-PID::BackwardSpeed + sSpeedAdjust);

        sDebugPhase = "Moving Back";
        break;
	}
    case csCollecting: //dbp2
    {
        // maintain roller speed
        if (UpdateTimer(diff, sRollerSampleTmr, ROLLER_SAMPLE_TIMER))
        {
            sLastMotorCount = GetAcqMotorCount(true);
            MaintainCollectionSpeed(diff, sLastMotorCount);
        }

		if (UpdateTimer(diff, sCollectTmr, cBallCollectionTimer))
		{
            sCollectionState = csWaitForBall;
			++sDebugPhase;
		}

        if (HasAcquired())
        {
			//exit cond
			robotState = rsNavigateForward;
			sCollectionState = csMoveBack;
			sCollectTmr = cBallCollectionTimer;
			//ShutdownAcqMotor(); //navigate forward will handle the shutdown
        }
        StopMoving();

        sDebugPhase = "Collecting";
        break;
    }
    case csWaitForBall:
        ActivateAcqMotor(cAcqMotorSpeed);

        if (sMoveForwardCompleted)
        {
			//dbp4
            StopMoving();
            if (UpdateTimer(diff, sWaitTmr, cWaitForBallTimer))
            {
                //exit condition
                sCollectionState = csMoveBack;
                sMoveForwardCompleted = false;
				sDebugPhase = "*****************";
            }
            sDebugPhase = "Waiting";
        }
        else
        {
			//dbp3
            if (UpdateTimer(diff, sMoveFwdTmr, cWaitForBallDistanceTmr))
            {				
                StopMoving();
                sMoveForwardCompleted = true;
            }
            MoveForward();
            sDebugPhase = "Moving Fwd";
        }
        break;
    }

    // update LCD
    if (UpdateTimer(diff, LCDTimer, LCD_UPDATE_TIMER))
    {
        // debug info here
        LCD_PRINTNEW("Collecting ");
		LCD.print(sDebugPhase);
        switch(sCollectionState)
        {
        case csMoveBack:
            LCD_PRINTBOT("Move back");
            break;
        case csCollecting:
            LCD_PRINTBOT("Roller: ");
            LCD.print(sLastMotorCount);
            break;
        case csWaitForBall:
            LCD_PRINTBOT("Wait");
            break;
        }
    }

    if (stopbutton())
    {
        SetConstant("QRD P Gain", PID::QRDPGain, 0.5f);
        SetConstant("QRD D Gain", PID::QRDDGain, 0.5f);
        SetConstant("Err Small", PID::ErrorSmall, 1/40.0f);
        SetConstant("Err Big", PID::ErrorBig, 1/40.0f);
		SetConstant("Ball Col Tmr", cBallCollectionTimer, 1000.0f);
        SetConstant("Speed", PID::BackwardSpeed, 0.5f);
    }
}

void MaintainCollectionSpeed(uint16 diff, uint16 motorCounts)
{
    int16 error = (int16)cRollerFreq - (int16)motorCounts;

    if (error > 0)
        ActivateAcqMotor(hMaxMotorSpeed);
    else
        ShutdownAcqMotor();
    LCD.print(cRollerFreq);
}