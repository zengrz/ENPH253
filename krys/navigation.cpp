#include "navigation_h.h"
#include "all_systems_go_h.h"
#include "running_h.h" // for updating robotState
#include "shoot.h" //for StartShooterControl()

static char* sDebugPhase = "******************";

static bool sDriveFwdCompleted = false;
static uint32 sAllowanceTmr = cPickupAllowanceTimer, sBackTmr = cBackStopTimer, sFwdDrvTmr = cNavFwdTimer;

//pre: at home collection area.
//post: some dist from the back (along center tape).
void NavigateForward(uint16 diff)
{
    StartShooterControl();
	if (UpdateTimer(diff, sAllowanceTmr, cPickupAllowanceTimer))
	{
		ShutdownAcqMotor();
	}

    float backIRThreash = sCalibratedBackIRVal * cIRFalloffCenter;

    int left = GetAveragedLeftRearOPReading();
    int right = GetAveragedRightRearOPReading();

    if (sDriveFwdCompleted)
    {
        if (UpdateTimer(diff, sBackTmr, cBackStopTimer))
        {
            StopMoving();
		    sAllowanceTmr = cPickupAllowanceTimer;
            sFwdDrvTmr = cNavFwdTimer;
		    robotState = rsShoot;
            sDriveFwdCompleted = false;
            sDebugPhase = "******************";
        }
        SetMotorSpeed(tLeftDriveMotor, -hMaxMotorSpeed);
        SetMotorSpeed(tRightDriveMotor, -hMaxMotorSpeed);
    }
    else
    {
        if (UpdateTimer(diff, sFwdDrvTmr, cNavFwdTimer)) //(left + right < backIRThreash)//(PID::OPDriveAdjustCompleted(backIRThreash))
            sDriveFwdCompleted = true;
        else
            PID::OPUpdate(false, diff, PID::OPDriveOrientP, 0, 0, cMoveFrontSpeed);
    }

    //PID::OPDriveUpdate(diff, backIRThreash);
    
    if (UpdateTimer(diff, LCDTimer, LCD_UPDATE_TIMER))
    {
        LCD_PRINTNEW("NF ");
        LCD.print(sDebugPhase);
        LCD.print(backIRThreash);
        
        LCD_PRINTBOT("IR: ");
        LCD.print(GetAveragedLeftRearOPReading());
        LCD.print(" ");
        LCD.print(GetAveragedRightRearOPReading());    
    }

    if (stopbutton())
    {
		SetConstant("Pickup Allowance", cPickupAllowanceTimer, 1000.0f);
        SetConstant("Center Falloff", cIRFalloffCenter, 1/1024.0f);
		SetConstant("Rear Diff Thresh", cRearAdjOPThresh, 1/20.0f);
        SetConstant("OP Drive Gain", PID::OPDriveP, 1/20.0f);
        SetConstant("OP Drive O Gain", PID::OPDriveOrientP, 1/20.0f);
    }

    sDebugPhase = "Moving ";
}

static NavigateBackState sNavigateBackState = nbAlignBack;

//pre: somewhere on the field (center, left or right).
//post: back OP lined up with 10kHz. robot on ceter tape.
void NavigateBackward(uint16 diff)
{
	switch (sNavigateBackState)
	{
	case nbAlignBack: //dbp1, should be redundant since shooter takes care of this. just checking for good measures.
		if (PID::RearOPAdjustCompleted())
            sNavigateBackState = nbFindTape;

		PID::OPUpdate(false, diff, PID::OPPGainBack, 0, 0);

        sDebugPhase = "Align Back";
		break;
	case nbFindTape: //dbp2
        if (RearLeftQRDOnTape() || RearRightQRDOnTape())
        {
			//exit robot state.
            StopMoving();
			sNavigateBackState = nbAlignBack;
			robotState = (IsLoaded()) ? rsNavigateForward : rsCollection; //should be empty as shooter only transfer control to here when it is not loaded.
            sDebugPhase = "******************";
        }

		PID::QRDUpdate(); // just turn, no move

        sDebugPhase = "Find Tape";
		break;
	}

    if (UpdateTimer(diff, LCDTimer, LCD_UPDATE_TIMER))
    {
        LCD_PRINTNEW("NB ");
        LCD.print(sDebugPhase);
        LCD_PRINTBOT("IR: ");
        LCD.print(GetLeftRearOPReading());
        LCD.print(" ");
        LCD.print(GetRightRearOPReading());
    }

	if (stopbutton())
	{
		SetConstant("Rear Diff Thresh", cRearAdjOPThresh, 1/20.0f);
		SetConstant("Rear OP P Gain", PID::OPPGainBack, 1/20.0f);
	}
}