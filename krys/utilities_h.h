#ifndef RUNNING_UTIILTIES_H_
#define RUNNING_UTIILTIES_H_

#include <phys253.h>
#include <servo253.h>
#include "configuration_h.h"

extern uint16 uCurLeftScaledMotorVel, uCurRightScaledMotorVel; //for debugging

//movement
void RotateCW(uint16 speed = cTurnSpeed);
void RotateCCW(uint16 speed = cTurnSpeed);
void MoveBackward(uint16 speed = cMoveBackSpeed);
void MoveForward(uint16 speed = cMoveFrontSpeed);
void StopMoving();
void MoveBackWhileNotOnTape(int numTapeToCross);
// protects to not give over max values
void SetMotorSpeed(int motorNum, int velocity); 
// pushes up min speeds so that it always drives
void SetMotorSpeedScaled(int motorNum, int velocity);

//shooter
void ActivateShooter(uint16 topSpeed = cShooterSpeedTop, uint16 botSpeed = cShooterSpeedBot);
void ShutdownShooter();
void GuardDown();
void GuardUp();

// shooter control
void InitShootFreqCounter();
void ResetShootFreqCounter();
void RegisterTopShootSwitchPressed();
void RegisterBotShootSwitchPressed();
int GetShootMotorCount(bool reset, bool top);

//acuisition control
void ActivateAcqMotor(int16 speed = cAcqMotorSpeed);
void ShutdownAcqMotor();

extern long rlrSwitchLastPressTime;
extern int rlrSwitchPressedCount;

void InitRollerFreqCounter();
void ResetRollerFreqCounter();
void RegisterRollerSwitchPressed();
int GetAcqMotorCount(bool reset);

//decisions (digital reads)
bool HasAcquired();
bool IsLoaded();
bool BackSwitchPressed();
bool RollerSwitchPressed();
bool RearLeftQRDOnTape();
bool RearRightQRDOnTape();

//analog reads
int GetLeftTargOPReading();
int GetRightTargOPReading();
int GetTargOPSum();
int GetLeftRearOPReading();
int GetRightRearOPReading();
int GetRearOPSum();
// averaged analog reads
int GetAveragedLeftRearOPReading();
int GetAveragedRightRearOPReading();
int GetAveragedRightTargOPReading();
int GetAveragedLeftTargOPReading();
int GetTargAveragedSum();
int GetRearAveragedSum();
void UpdateOPAverages();
void InitOPAverages();
void UpdateOPAverage(float& average, int* avgArray, int& currUpdate, int reading);

// returns true if time has run out
bool UpdateTimer(uint16 diff, uint32& timer, uint32 resetVal);

//miscellaneous
template<typename T>
T SetConstant(char* name, T& def, float multiplier = 1)
{
    while (startbutton() || stopbutton()) {}
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print(name);
    LCD.setCursor(0, 1);
    LCD.print(def);

    bool isSet = false;
    T val;

    while (true)
    {
        if (stopbutton())
        {
            val = (T)def;
            isSet = true;
            break;
        }
        if (startbutton())
            break;
    }

    while (stopbutton() || startbutton()) {}

    if (!isSet)
    {
        while (!stopbutton())
        {
            LCD.clear();
            LCD.setCursor(0, 0);
            LCD.print(name);
            LCD.setCursor(0, 1);
            LCD.print(knob(6) * multiplier);
            delay(LCD_UPDATE_TIMER / 1000.0f);
        }
        val = (T)(knob(6) * multiplier);
    }

    while (startbutton() || stopbutton()) {}
    def = val;
    return val;
}

#endif