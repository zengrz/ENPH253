#include "all_systems_go_h.h"

void AllSystemsGo()
{
    portMode(0, INPUT);
    portMode(1, INPUT);
  
    RCServo0.detach(); 
    RCServo1.detach();
    RCServo2.attach(RCServo2Output);

    GuardUp();

    InitRollerFreqCounter();
    InitShootFreqCounter();
    InitOPAverages();
    PID::QRDReset();
    PID::OPReset();    
}