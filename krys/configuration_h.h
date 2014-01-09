#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

typedef char   int8;
typedef int    int16;
typedef long   int32;

typedef unsigned char   uint8;
typedef unsigned int    uint16;
typedef unsigned long   uint32;

//TINAH pin configurations
//analog
extern int tLeftTargetOP;
extern int tRightTargetOP;
extern int tLeftRearOP;
extern int tRightRearOP;
//digital
extern int tAcqMotorFreqRecorder;
extern int tTopShootQRD;
extern int tBotShootQRD;
extern int tRearLeftQRD;
extern int tRearRightQRD;
extern int tGuardQRD;
extern int tRearSwitch;
//motor
extern int tLeftDriveMotor;
extern int tRightDriveMotor;
//PWM
extern int tCollectorMotor;
extern int tShooterMotorTop;
extern int tShooterMotorBot;

//hardware configurations
extern int hMinFrontMotorSpeed;
extern int hMinBackMotorSpeed;
extern int hRestMotorSpeed;
extern int hMaxMotorSpeed;
extern int hServoCenterAngle;

//robot configurations
// init
extern int cShakeCount;

// aiming
extern int cTargOPMinThresh;
extern int cTargOPDiffThresh;
extern int cRearAdjOPThresh;
extern int cRearOPMinThresh;
extern float cFrontRightOPGain;
extern float cTurnIRFalloffCW;
extern float cTurnIRFalloffCCW;
extern int cAimSpeed;
extern int cAimSwitchThreshold;

// shooter
extern int cShooterSpeedTop;
extern int cShooterSpeedBot;
extern int cShooterTopFreq;
extern int cShooterBotFreq;
extern float cShooterIntegral;
extern float cShooterGain;

// default motor speeds
extern int cAcqMotorSpeed;

// servo
extern int cGuardAngleAcquire;
extern int cGuardAngleRelease;

// roller control
extern int cRollerDelay;
extern int cRollerFreq;
extern int cRollerProportion;
extern float cRollerIntegral;

// navigation
extern float cRearRightOPGain;
extern int cMoveFrontSpeed;
extern int cMoveBackSpeed;
extern int cTurnSpeed;
extern float cIRFalloffCenter;

namespace PID
{
    //PID movement
    extern int QRDPGain, QRDDGain;
    extern int BackwardSpeed;
    extern float ErrorSmall, ErrorBig;

    //PID movement
    extern float OPPGainFront;
    //back align
    extern float OPPGainBack;
    //drive
    extern int OPDriveOrientP;
    extern int OPDriveP;
}

extern int16 sCalibratedBackIRVal;

//timers
#define TIMER_MIN_DELAY_THRESH  10000
#define S_TO_US                 1000000 // seconds to micro seconds
#define LCD_UPDATE_TIMER        100000 // 100 milliseconds
#define ROLLER_SAMPLE_TIMER        ((uint32)100*(uint32)1000) // 100 ms (min 10 Hz)

//lcd
extern uint32 LCDTimer;

//init
extern uint32 cShakeTimer;

//nav
extern uint32 cBackStopTimer;
extern uint32 cNavFwdTimer;

//shoot
extern uint32 cShooterUpdateTimer;
extern uint32 cShootTimer;
extern uint32 cSearchTimer;
extern uint32 cLoadTimer;
extern uint32 cTurnRecoverTimer;
extern uint32 cAimTurnRecoverTimer;
extern uint32 cShootCooldownTimer;
extern uint32 cMaxAimDurationTimer;
extern uint32 cCheckAimTimer;

//acq
extern uint32 cBallCollectionTimer;
extern uint32 cPickupAllowanceTimer;
extern uint32 cWaitForBallTimer;
extern uint32 cWaitForBallDistanceTmr;

#endif