#include "pid_system_h.h"
#include "utilities_h.h" // for TINAH accessor functions

//PID QRD
float PID::QRDLastError;
uint16 PID::QRDErrorTime;

void PID::QRDReset()
{
    PID::QRDLastError = 0.0f;
    PID::QRDErrorTime = 0;
}

// updated as the robot is running to keep track of the tape.
void PID::QRDUpdateLastError()
{
    PID::QRDLastError = PID::GetQRDError();
}

float PID::GetQRDError()
{
    float error = 0;
    bool leftQRDOnTape = RearLeftQRDOnTape();
    bool rightQRDOnTape = RearRightQRDOnTape();

    if (leftQRDOnTape && rightQRDOnTape)
    {
        error = 0;
    }
    else if (leftQRDOnTape && !rightQRDOnTape)
    {
        error = -PID::ErrorSmall;
    }
    else if (!leftQRDOnTape && rightQRDOnTape)
    {
        error = +PID::ErrorSmall;
    }
    else if (!leftQRDOnTape && !rightQRDOnTape) 
    {
        error = ((PID::QRDLastError > 0) ? PID::ErrorBig : -PID::ErrorBig);
    }
    return error;
}

int16 PID::GetQRDAdjustment()
{
    float curError = PID::GetQRDError();
    uint16 timeSinceErrorChanged = PID::QRDErrorTime;

    if (curError != PID::QRDLastError) 
    {
        PID::QRDErrorTime = 1;
    }

    float pAdj = PID::QRDPGain * curError;
    float dAdj = ((float)PID::QRDDGain * (float)(curError - PID::QRDLastError) / (float)(PID::QRDErrorTime + timeSinceErrorChanged));

    PID::QRDErrorTime++;
    PID::QRDLastError = curError;
    return (int16)(pAdj + dAdj);
}

void PID::QRDUpdate(int16 vel) // vel should always be negative
{
    int16 speedAdj = PID::GetQRDAdjustment();

	//int16 adjustedLeftVel = (vel - speedAdj) > 0 ? 0 : (vel - speedAdj);
	//int16 adjustedRightVel = (vel + speedAdj) > 0 ? 0 : (vel + speedAdj);

    int16 adjustedLeftVel = (vel - speedAdj);
    int16 adjustedRightVel = (vel + speedAdj);

    SetMotorSpeedScaled(tLeftDriveMotor, adjustedLeftVel);
    SetMotorSpeedScaled(tRightDriveMotor, adjustedRightVel);
}

//PID OP
float PID::OPLastError, PID::OPTotalError;
uint16 PID::OPMaxIntegral = 100000;

void PID::OPReset()
{
    PID::OPTotalError = PID::OPLastError = 0;
}

int PID::GetOPError(bool front, int16 lastError, float rearLeftOPOffsetGain, float rearRightOPOffsetGain)
{
    int16 leftReading = (front ? GetAveragedLeftTargOPReading() : GetAveragedLeftRearOPReading());
    int16 rightReading = (front ? GetAveragedRightTargOPReading() : GetAveragedRightRearOPReading());
    float error = rightReading - leftReading;

    int16 thresh = front ? cTargOPMinThresh : cRearOPMinThresh;
    if (leftReading < thresh && rightReading < thresh)
        error = (lastError > 0) ? 1 : -1;
    else
        error /= ((float)(leftReading + rightReading));
    
    return error * 100; //about 0 - 70, 100 if both OPs have readings too low.
}

int16 PID::GetOPAdjustment(bool front, uint16 diff, float p, float i, float d, float rearLeftOPOffsetGain, float rearRightOPOffsetGain)
{
    int curOPError = PID::GetOPError(front, PID::OPLastError, rearLeftOPOffsetGain, rearRightOPOffsetGain);
    //need to include this in the most recent adjustment to make corrections quickly (and prevent error from getting big)
    //i
    PID::OPTotalError += curOPError * diff;
    if (PID::OPTotalError > PID::OPMaxIntegral)
        PID::OPTotalError = PID::OPMaxIntegral;
    else if (PID::OPTotalError < -PID::OPMaxIntegral)
        PID::OPTotalError = -PID::OPMaxIntegral;

    int16 pAdj = p * curOPError;
    int16 iAdj = i * PID::OPTotalError;
    int16 dAdj = ((float)d * (float)(curOPError - PID::OPLastError) / (float)(diff));

    PID::OPLastError = curOPError;
    return (pAdj + iAdj + dAdj);
}

void PID::OPUpdate(bool frontOP, uint16 diff, float p, float i, float d, int16 vel, float rearLeftOPOffsetGain, float rearRightOPOffsetGain)
{
    int speedAdj = PID::GetOPAdjustment(frontOP, diff, p, i, d, rearLeftOPOffsetGain, rearRightOPOffsetGain);

    SetMotorSpeedScaled(tLeftDriveMotor, vel + speedAdj * (frontOP ? 1 : -1));
    SetMotorSpeedScaled(tRightDriveMotor, vel + speedAdj * (frontOP ? -1 : 1));
}

bool PID::TargOPAdjustCompleted()
{
    int leftReading = GetAveragedLeftTargOPReading();
    int rightReading = GetAveragedRightTargOPReading();
    int diff = leftReading - rightReading;

    return (abs(diff) < cTargOPDiffThresh) && (leftReading + rightReading > cTargOPMinThresh);
}

bool PID::RearOPAdjustCompleted(float leftGain, float rightGain)
{
    int leftReading = GetAveragedLeftRearOPReading() * leftGain;
    int rightReading = GetAveragedRightRearOPReading() * rightGain;
    int diff = leftReading - rightReading;

    return (abs(diff) < cRearAdjOPThresh);
}

void PID::OPDriveUpdate(uint16 diff, uint16 goalRearIRThresh)
{
    int16 error = GetRearAveragedSum() - goalRearIRThresh;

    int16 vel = PID::OPDriveP * error; //change this to a constant val?
    PID::OPUpdate(false, diff, PID::OPDriveOrientP, 0, 0, vel);
}

bool PID::OPDriveAdjustCompleted(uint16 goalRearIRThresh, float leftGain, float rightGain)
{
    int left = (float)GetAveragedLeftRearOPReading() * (float)leftGain;
    int right = (float)GetAveragedRightRearOPReading() * (float)rightGain;
    return abs(left + right - goalRearIRThresh) < cRearAdjOPThresh 
        && abs(left - right) < cRearAdjOPThresh;
}