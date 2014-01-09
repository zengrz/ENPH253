#include "shoot.h"
#include "all_systems_go_h.h"
#include "running_h.h"

static char* sDebugPhase = "******************";

static ShooterState shooterState = ssAim; // used throghout
static AimState aimState = asCW;
static uint32 loadTimer = cLoadTimer;
static uint32 searchTimer = cSearchTimer;
static uint32 sShootTmr = cShootTimer;
static int16 beginBackIR = -1;
static int16 frontIRMax = -1;
static int16 lastTargReading;
static uint16 aimSwitchCount = 0;
static uint32 turnRecoverTimer = cTurnRecoverTimer;
static uint16 shooterTopFreq = cShooterTopFreq;
static uint16 shooterBotFreq = cShooterBotFreq;
static bool turnRecovered = false;
static uint32 aimTurnRecoverTimer = cAimTurnRecoverTimer;
static bool aimTurnRecovered = false;
static bool shooterReady = false;
static bool coarseMaxThreshPassed = false;
static bool doMidAdjust = false;
static bool midAjustComplete = true;
static uint32 sAimTmr = cMaxAimDurationTimer;

void Shoot(uint16 diff)
{
    switch (shooterState)
    {
    case ssAim: // controls if robot is to shoot or to move back
        StartShooterControl();
        if (beginBackIR == -1)
            beginBackIR = GetRearAveragedSum();
        // try to load more balls
        if (!IsLoaded())
        {
            StopMoving();
            GuardUp();
            if (UpdateTimer(diff, loadTimer, cLoadTimer)) // ran out of time to load new ball - have no balls
                shooterState = ssAlignBack;
            sDebugPhase = "Loading";
        }
        else
        {
            loadTimer = cLoadTimer;
            if (GetTargAveragedSum() < cTargOPMinThresh) //need to coarse adj
            {
                if (doMidAdjust)
                {
                    StopMoving();
                    doMidAdjust = false;
                    midAjustComplete = true;
                    /*
                    if (PID::OPDriveAdjustCompleted(sCalibratedBackIRVal * cIRFalloffCenter))
                    {
                        StopMoving();
                        doMidAdjust = false;
                        midAjustComplete = true;
                    }

                    PID::OPDriveUpdate(diff, sCalibratedBackIRVal * cIRFalloffCenter);
                    */
                }
                else
                {
                    if (aimState == asCW)
                    {
                        //RotateCW(cAimSpeed);
                        SetMotorSpeed(tLeftDriveMotor, 100 + cAimSpeed);
                        SetMotorSpeed(tRightDriveMotor, 100 - cAimSpeed);
                    }
                    else if (aimState == asCCW)
                    {
                        //RotateCCW(cAimSpeed);
                        SetMotorSpeed(tLeftDriveMotor, 100 - cAimSpeed);
                        SetMotorSpeed(tRightDriveMotor, 100 + cAimSpeed);
                    }
                }
                
                if (coarseMaxThreshPassed) //coarse adj control
                {
                    if (GetRearAveragedSum() < beginBackIR * (aimState == asCW ? cTurnIRFalloffCW : cTurnIRFalloffCCW))
                    {
                        //switch
                        aimState = (aimState == asCW ? asCCW : asCW);
                        coarseMaxThreshPassed = false;
                        midAjustComplete = false;
                        doMidAdjust = false;
                    }
                    sDebugPhase = "Coarse 2";
                }
                else 
                {
                    if (GetRearAveragedSum() >= beginBackIR * 0.75f)
                    {
                        if (!midAjustComplete)
                            doMidAdjust = true;
                        
                        coarseMaxThreshPassed = true;
                    }
                    sDebugPhase = "Coarse 1";
                }
            }
            else // fine
            {
                midAjustComplete = false;
                doMidAdjust = false;
                if (GetRearAveragedSum() >= beginBackIR * 0.75f)
                    coarseMaxThreshPassed = true;
                if (PID::TargOPAdjustCompleted())
                {
                    StopMoving();
                    shooterState = ssFire;
                    //sAimTmr = cMaxAimDurationTimer;
                }

                PID::OPUpdate(true, diff, PID::OPPGainFront, 0, 0);

                if (IsLoaded())
                    if (UpdateTimer(diff, sAimTmr, cMaxAimDurationTimer))
                    {
                        StopMoving();
                        shooterState = ssFire;
                    }

                sDebugPhase = "Fine Adj";
            }
        }
        break;
    
    case ssFire:
        StopMoving();
        
        if (shooterReady)
            GuardDown();

        if (!IsLoaded())
            if (UpdateTimer(diff, sShootTmr, cShootTimer))
            {
                sAimTmr = cMaxAimDurationTimer;
                shooterState = ssAim;
            }

        sDebugPhase = "Fire";
        break;

    case ssAlignBack:
		StopShooterControl();

        if (PID::RearOPAdjustCompleted())
        {
            StopMoving();
            shooterState = ssAim;
            robotState = rsNavigateBack;
            beginBackIR = -1;
            sDebugPhase = "******************";
        }

        PID::OPUpdate(false, diff, PID::OPPGainBack, 0, 0);
        sDebugPhase = "Align Back";
        break;
    }

    if (UpdateTimer(diff, LCDTimer, LCD_UPDATE_TIMER))
    {
        LCD_PRINTNEW("S:");
        LCD.print(sDebugPhase);
        LCD.setCursor(0, 1);
        switch (shooterState)
        {
        case ssAim:
            LCD.print("A ");
            LCD.print(GetAveragedLeftTargOPReading());
            LCD.print(" ");
            LCD.print(GetAveragedRightTargOPReading());
            LCD.print(" ");
            LCD.print(beginBackIR * cTurnIRFalloffCW);
            LCD.print(" ");
            LCD.print(GetRearAveragedSum());
            break;
        case ssFire:
            LCD.print("SPD ");
            LCD.print(shooterTopFreq);
            LCD.print(" ");
            LCD.print(shooterBotFreq);
            LCD.print(" ");
            break;
        case ssAlignBack:
            LCD.print("B ALGN ");
            LCD.print(GetLeftRearOPReading());
            LCD.print(" ");
            LCD.print(GetRightRearOPReading());
            LCD.print(" ");
            break;
        }
        
    }

    if (stopbutton())
    {
        StopShooterControl();
        shooterTopFreq = SetConstant("Shooter Top Freq", cShooterTopFreq, 1/2.0f);
        shooterBotFreq = SetConstant("Shooter Bot Freq", cShooterBotFreq, 1/2.0f);
        SetConstant("Right OP Gain", cFrontRightOPGain, 1/20.0f);
        StartShooterControl();
    }
}

static bool activated = false;
static uint32 shooterUpdateTimer = cShooterUpdateTimer;
static float shooterTopSpeed = shooterTopFreq * 4.0f * cShooterUpdateTimer / 1000000.0f;
static float shooterBotSpeed = shooterBotFreq * 4.0f * cShooterUpdateTimer / 1000000.0f;

static float shooterTopIntegral = 0, shooterBotIntegral = 0;

void UpdateShooter(uint16 diff)
{
    shooterTopSpeed = shooterTopFreq * 4.0f * (float)cShooterUpdateTimer / 1000000.0f;
    shooterBotSpeed = shooterBotFreq * 4.0f * (float)cShooterUpdateTimer / 1000000.0f;
    if (!activated)
        return;
    if (UpdateTimer(diff, shooterUpdateTimer, cShooterUpdateTimer))
    {
        int topCount = GetShootMotorCount(true, true);
        int botCount = GetShootMotorCount(true, false);

        float errorTop = shooterTopSpeed - topCount;
        float errorBot = shooterBotSpeed - botCount;

        shooterTopIntegral += errorTop * (float)cShooterUpdateTimer / 1000000.0f;
        shooterBotIntegral += errorBot * (float)cShooterUpdateTimer / 1000000.0f;

        if (shooterTopIntegral > 35)
            shooterTopIntegral = 35;
        if (shooterTopIntegral < -35)
            shooterTopIntegral = 35;
        if (shooterBotIntegral > 35)
            shooterBotIntegral = 35;
        if (shooterBotIntegral < -35)
            shooterBotIntegral = 35;

        int topSpeed = errorTop * cShooterGain + shooterTopIntegral * cShooterIntegral;
        int botSpeed = errorBot * cShooterGain + shooterBotIntegral * cShooterIntegral;

        if (errorTop < 0) topSpeed = 0;
        if (errorBot < 0) botSpeed = 0;
        if (topSpeed > hMaxMotorSpeed)
            topSpeed = hMaxMotorSpeed;
        if (botSpeed > hMaxMotorSpeed)
            botSpeed = hMaxMotorSpeed;

        ActivateShooter(topSpeed, botSpeed);

        shooterReady = (abs(errorTop) < 5 && abs(errorBot) < 5);
        
        /*LCD.clear(); LCD.home();
        LCD.print(topCount);
        LCD.print(" ");
        LCD.print(botCount);
        LCD.setCursor(0, 1);

        LCD.print(shooterTopIntegral);
        LCD.print(" ");
        LCD.print(shooterBotIntegral);*/
    }
}

void StopShooterControl()
{
    activated = false;
    ShutdownShooter();
}

bool ShooterActivated()
{
    return activated;
}

void StartShooterControl()
{
    activated = true;
}