#include "utilities_h.h"

//execution functions (analog/digital write)

//movement

void RotateCW(uint16 speed)
{
    motor.speed(tRightDriveMotor, -speed);
    motor.speed(tLeftDriveMotor, speed);
}

void RotateCCW(uint16 speed)
{
    motor.speed(tLeftDriveMotor, -speed);
    motor.speed(tRightDriveMotor, speed);
}

void MoveBackward(uint16 speed)
{
    motor.speed(tLeftDriveMotor, -speed);
    motor.speed(tRightDriveMotor, -speed);
}

void MoveForward(uint16 speed)
{
    motor.speed(tLeftDriveMotor, speed);
    motor.speed(tRightDriveMotor, speed);
}

void StopMoving()
{
    motor.speed(tLeftDriveMotor, hRestMotorSpeed);
    motor.speed(tRightDriveMotor, hRestMotorSpeed);
}

void MoveBackWhileNotOnTape(int numTapeToCross)
{
    MoveBackward();
    for (int i = 0; i < numTapeToCross; i++)
    {
        while ((!RearLeftQRDOnTape() && !RearRightQRDOnTape())) {}
    }
    StopMoving();
}

void SetMotorSpeed(int motorNum, int velocity)
{
    velocity = ((velocity > hMaxMotorSpeed) ? hMaxMotorSpeed : velocity);
    velocity = ((velocity < -hMaxMotorSpeed) ? -hMaxMotorSpeed : velocity);
    motor.speed(motorNum, velocity);
}

uint16 uCurLeftScaledMotorVel = 0, uCurRightScaledMotorVel = 0;

// will cause problem in the case of pure rotation. one wheel has to rotate forward while the other backward, unless hMinFrontMotorSpeed and hMinBackMotorSpeed have same values.
void SetMotorSpeedScaled(int motorNum, int vel)
{
    if (abs(vel) < 2)
        vel = 0;
    else
        vel = (vel > 0) ?
        vel * (1 - (float)hMinFrontMotorSpeed / (float)hMaxMotorSpeed) + hMinFrontMotorSpeed :
        vel * (1 - (float)hMinBackMotorSpeed / (float)hMaxMotorSpeed) - hMinBackMotorSpeed;

	//for debugging
	if (motorNum == tLeftDriveMotor)
		uCurLeftScaledMotorVel = vel;
	else if (motorNum == tRightDriveMotor)
		uCurRightScaledMotorVel = vel;

    SetMotorSpeed(motorNum, vel);
}

//shooter

void ActivateShooter(uint16 topSpeed, uint16 botSpeed) //activate the two H-bridges controlling the shooter motors.
{
    analogWrite(tShooterMotorTop, topSpeed);
    analogWrite(tShooterMotorBot, botSpeed);
}

void ShutdownShooter() //activate the two H-bridges controlling the shooter motors.
{
    analogWrite(tShooterMotorTop, hRestMotorSpeed);
    analogWrite(tShooterMotorBot, hRestMotorSpeed);
}

void GuardDown() //shoot
{
    RCServo2.write(cGuardAngleRelease);
}

void GuardUp() //load
{
    RCServo2.write(cGuardAngleAcquire);
}

static long topLastTime;
static int topSwitchCount;

static long botLastTime;
static int botSwitchCount;

// shooter control
void InitShootFreqCounter()
{
    attachInterrupt(tTopShootQRD, RegisterTopShootSwitchPressed, RISING);
    attachInterrupt(tBotShootQRD, RegisterBotShootSwitchPressed, RISING);
    ResetRollerFreqCounter();
}

void ResetShootFreqCounter()
{
    topLastTime = millis();
    botLastTime = millis();

    topSwitchCount = 0;
    botSwitchCount = 0;
}

void RegisterTopShootSwitchPressed()
{
    long temp = millis();
    if (temp > topLastTime + 2)
    {
        ++topSwitchCount;
        topLastTime = temp;
    }
}

void RegisterBotShootSwitchPressed()
{
    long temp = millis();
    if (temp > botLastTime + 2)
    {
        ++botSwitchCount;
        botLastTime = temp;
    }
}

int GetShootMotorCount(bool reset, bool top)
{
    int ret = top ? topSwitchCount : botSwitchCount;
    if (reset)
        if (top)
            topSwitchCount = 0;
        else
            botSwitchCount = 0;
    return ret;
}

//acquisition

void ActivateAcqMotor(int16 speed)
{
    if (speed < 0) speed = 0;
    if (speed > hMaxMotorSpeed) speed = hMaxMotorSpeed;
    analogWrite(tCollectorMotor, speed);
}

void ShutdownAcqMotor()
{
    analogWrite(tCollectorMotor, hRestMotorSpeed);
}

long rlrSwitchLastPressTime;
int rlrSwitchPressedCount;

void InitRollerFreqCounter()
{
    attachInterrupt(tAcqMotorFreqRecorder, RegisterRollerSwitchPressed, FALLING);
    ResetRollerFreqCounter();
}

void ResetRollerFreqCounter()
{
    rlrSwitchLastPressTime = millis();
    rlrSwitchPressedCount = 0;
}

void RegisterRollerSwitchPressed()
{
    long temp = millis();
    if (temp > rlrSwitchLastPressTime + 2)
    {
        ++rlrSwitchPressedCount;
        rlrSwitchLastPressTime = temp;
    }
}

int GetAcqMotorCount(bool reset)
{
    int ret = rlrSwitchPressedCount;
    if (reset)
        rlrSwitchPressedCount = 0;
    return ret;
}

//decision functions (digital read)

bool HasAcquired() //check if ball has been picked up using the sensor at the holding area
{
    return !digitalRead(tGuardQRD);
}

bool IsLoaded() //basically the same function as "ballsAcquired" but used when try to reload after firing. only change may be the delay.
{
    HasAcquired();
}

bool BackSwitchPressed()
{
    return !digitalRead(tRearSwitch);
}

bool RollerSwitchPressed()
{
    return !digitalRead(tAcqMotorFreqRecorder);
}

bool RearLeftQRDOnTape()
{
    return digitalRead(tRearLeftQRD);
}

bool RearRightQRDOnTape()
{
    return digitalRead(tRearRightQRD);
}

//analog reads

int GetLeftTargOPReading()
{
    return analogRead(tLeftTargetOP);
}

int GetRightTargOPReading()
{
    return int(float(analogRead(tRightTargetOP) * cFrontRightOPGain));
}

int GetTargOPSum()
{
    return GetLeftTargOPReading() + GetRightTargOPReading();
}

int GetLeftRearOPReading()
{
    return analogRead(tLeftRearOP);
}

int GetRightRearOPReading()
{
    return analogRead(tRightRearOP) * cRearRightOPGain;
}

int GetRearOPSum()
{
    return GetLeftRearOPReading() + GetRightRearOPReading();
}

#define OP_AVERAGE_COUNT 15
static int leftRearOPReadings[OP_AVERAGE_COUNT];
static int rightRearOPReadings[OP_AVERAGE_COUNT];
static int rightTargOPReadings[OP_AVERAGE_COUNT];
static int leftTargOPReadings[OP_AVERAGE_COUNT];
static int currLeftRearUpdate = 0, currRightRearUpdate = 0, currRightTargUpdate = 0, currLeftTargUpdate = 0;
static float leftRearAverage, rightRearAverage, rightTargAverage, leftTargAverage;

void InitOPAverages()
{
    int left = GetLeftRearOPReading();
    int right = GetRightRearOPReading();
    int targRight = GetRightTargOPReading();
    int targLeft = GetLeftTargOPReading();
    for (int i = 0; i < OP_AVERAGE_COUNT; ++i)
    {
        leftRearOPReadings[i] = left;
        rightRearOPReadings[i] = right;
        rightTargOPReadings[i] = targRight;
        leftTargOPReadings[i] = targLeft;
    }
    leftRearAverage = left;
    rightRearAverage = right;
    rightTargAverage = targRight;
    leftTargAverage = targLeft;
}

void UpdateOPAverages()
{
    int left = GetLeftRearOPReading();
    int right = GetRightRearOPReading();
    int targRight = GetRightTargOPReading();
    int targLeft = GetLeftTargOPReading();
    UpdateOPAverage(leftRearAverage, leftRearOPReadings, currLeftRearUpdate, left);
    UpdateOPAverage(rightRearAverage, rightRearOPReadings, currRightRearUpdate, right);
    UpdateOPAverage(rightTargAverage, rightTargOPReadings, currRightTargUpdate, targRight);
    UpdateOPAverage(leftTargAverage, leftTargOPReadings, currLeftTargUpdate, targLeft);
}

void UpdateOPAverage(float& average, int* avgArray, int& currUpdate, int reading)
{
    avgArray[currUpdate] = reading;
    ++currUpdate;
    if (currUpdate >= OP_AVERAGE_COUNT)
        currUpdate = 0;
    // recalc average
    average = 0;
    for (int i = 0; i < OP_AVERAGE_COUNT; ++i)
        average += avgArray[i];
    average /= OP_AVERAGE_COUNT;
}

int GetAveragedLeftRearOPReading()
{
    return (int)(leftRearAverage + 0.5);
}

int GetAveragedRightRearOPReading()
{
    return (int)(rightRearAverage + 0.5);
}

int GetRearAveragedSum()
{
    return GetAveragedLeftRearOPReading() + GetAveragedRightRearOPReading();
}

int GetAveragedRightTargOPReading()
{
    return (int)(rightTargAverage + 0.5);
}

int GetAveragedLeftTargOPReading()
{
    return (int)(leftTargAverage + 0.5);
}

int GetTargAveragedSum()
{
    return GetAveragedRightTargOPReading() + GetAveragedLeftTargOPReading();
}

//exception handling functions

// timer
bool UpdateTimer(uint16 diff, uint32& timer, uint32 resetVal)
{
    if (timer < diff)
    {
        timer = resetVal;
        return true;
    }
    timer -= diff;
    return false;
}