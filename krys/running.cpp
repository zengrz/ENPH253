#include "running_h.h"
#include "collection_h.h"
#include "navigation_h.h"
#include "shoot.h"

RobotState robotState = rsInit;
static bool sShakeCompleted = false;
static uint16 shakeCount = 0;
static uint32 shakeTimer = cShakeTimer;
static TakeoffState takeoffState = tsShakeForwards;

void Takeoff(uint16 diff) // navigate to center tape. start at audience's side
{
    if (!sShakeCompleted)
    {
        // shake robot to lower back touch sensor and front guard
        switch (takeoffState)
        {
        case tsShakeForwards:
            MoveForward(hMaxMotorSpeed);
            if (UpdateTimer(diff, shakeTimer, cShakeTimer))
            {
                ++shakeCount;
                takeoffState = tsShakeBackwards;
            }
        
            break;
        case tsShakeBackwards:
            MoveBackward(hMaxMotorSpeed);
            if (UpdateTimer(diff, shakeTimer, cShakeTimer))
            {
                ++shakeCount;
                takeoffState = tsShakeForwards;
            }
            break;
        }

        if (shakeCount >= cShakeCount)
        {
		    StopMoving();
            shakeCount = 0;
            sShakeCompleted = true;
        }
    }
    else
    {
        if (RearLeftQRDOnTape() || RearRightQRDOnTape())
        {
            StopMoving();
            sShakeCompleted = false;
            robotState = rsCollection;
        }

        MoveBackward(cMoveBackSpeed + 150);
    }
}

static uint32 sLastUpdate = micros();

void RunRobot(bool debug, RobotState debugState)
{
    uint32 diff32 = (uint32)micros() - (uint32)sLastUpdate;
    sLastUpdate = micros();

    if (diff32 > TIMER_MIN_DELAY_THRESH)
        diff32 = TIMER_MIN_DELAY_THRESH;

    uint16 diff = (uint16)diff32;

    switch(robotState)
    {
    case rsInit:
        // go to center tape
        Takeoff(diff);
        break;
    case rsCollection:
        Collect(diff);
        break;
    case rsNavigateForward:
        NavigateForward(diff);
        break;
    case rsShoot:
        Shoot(diff);
        break;
    case rsNavigateBack:
        NavigateBackward(diff);
        break;
    default:
        break;
    }

    UpdateShooter(diff);
    UpdateOPAverages();
    PID::QRDUpdateLastError();
  
    if (debug && robotState != debugState)
    {
        LCD_PRINTNEW("Start to reset");
        ShutdownAcqMotor();
        DEBUG_PAUSE
        robotState = debugState;
    }
}