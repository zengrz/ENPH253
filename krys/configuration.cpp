#include "configuration_h.h"

//TINAH pin constants
//analog
int tLeftTargetOP = 0;
int tRightTargetOP = 1;
int tRightRearOP = 2;
int tLeftRearOP = 3;
//digital
int tAcqMotorFreqRecorder = 0;
int tTopShootQRD = 1;
int tBotShootQRD = 2;
int tRearRightQRD = 3;
int tRearLeftQRD = 4;
int tGuardQRD = 5;
int tRearSwitch = 6;
//motor
int tLeftDriveMotor = 0;
int tRightDriveMotor = 1;
//PWM
int tCollectorMotor = 1;
int tShooterMotorTop = 2;
int tShooterMotorBot = 3;

//hardware configurations
int hRestMotorSpeed = 0;
int hMaxMotorSpeed = 1023;
int hServoCenterAngle = 90;
int hMinFrontMotorSpeed = 300;
int hMinBackMotorSpeed = 300;

//robot configurations
//init
int cShakeCount = 4;

//aim
int cTargOPMinThresh = 15;
int cTargOPDiffThresh = 4;
int cRearAdjOPThresh = 5;
int cRearOPMinThresh = 10;
//float cFrontRightOPGain = 2.45f;
float cFrontRightOPGain = 0.7f;
float cTurnIRFalloffCW = 0.60f;
float cTurnIRFalloffCCW = 0.40f;
int cAimSpeed = 300;
int cAimSwitchThreshold = 4;

//shooter
int cShooterSpeedTop = 30;
int cShooterSpeedBot = 50;
int cShooterTopFreq = 30; // Hz
int cShooterBotFreq = 30;
float cShooterIntegral = 1;
float cShooterGain = 500;

//acquisition
int cAcqMotorSpeed = 600;
int cGuardAngleAcquire = 112;
int cGuardAngleRelease = 39;
// roller control
int cRollerDelay = 2500;
int cRollerFreq = (70 * (float)(ROLLER_SAMPLE_TIMER) / (1000.0f * 1000.0f)); // 35 Hz
int cRollerProportion = 130;
float cRollerIntegral = 0.1f;
// navigation
float cRearRightOPGain = 0.93f;
int cMoveFrontSpeed = 700;
int cMoveBackSpeed = 300;
int cTurnSpeed = 300;
float cIRFalloffCenter = 0.5f;

int PID::QRDPGain = 93;
int PID::QRDDGain = 33;
int PID::BackwardSpeed = 115;
float PID::ErrorSmall = 1.0f;
float PID::ErrorBig = 2.0f;
//PID OP configurations
//float PID::OPPGainFront = 3.5f;
float PID::OPPGainFront = 2.0f;
float PID::OPPGainBack = 4.5f; // can be higher. max adj at 300, but scaled by 
// drive
int PID::OPDriveP = 12;
int PID::OPDriveOrientP = 11;

// etc
int16 sCalibratedBackIRVal = 393;

// timers (in microseconds)
//lcd
uint32 LCDTimer = 10 * (uint32)1000;

//init
uint32 cShakeTimer = 100*(uint32)1000;

//nav
uint32 cBackStopTimer = 100 * (uint32)1000;
uint32 cNavFwdTimer = 1100 * (uint32)1000;

//shooter
uint32 cShooterUpdateTimer = 44 * (uint32)1000;
uint32 cShootTimer = 1300 * (uint32)1000;
uint32 cSearchTimer = 3500 * (uint32)1000;
uint32 cLoadTimer = 1000 * (uint32)1000;
uint32 cTurnRecoverTimer = 500 * (uint32)1000;
uint32 cAimTurnRecoverTimer = 200 * (uint32)1000;
uint32 cShootCooldownTimer = 1000 * (uint32)1000;
uint32 cMaxAimDurationTimer = 3000 * (uint32)1000;
uint32 cCheckAimTimer = 150 * (uint32)1000;

//acq
uint32 cBallCollectionTimer = 3000 * (uint32)1000;
uint32 cPickupAllowanceTimer = 1000 * (uint32)1000;
uint32 cWaitForBallTimer = 1000 * (uint32)1000;
uint32 cWaitForBallDistanceTmr = 1000 * (uint32)1000;