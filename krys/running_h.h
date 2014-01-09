#ifndef RUNNING_H_
#define RUNNING_H_

#include "all_systems_go_h.h"

enum TakeoffState
{
    tsShakeForwards,
    tsShakeBackwards,
};

//robot state (shoot, acq., etc...)
enum RobotState
{
    rsInit = 0,
    rsCollection,         // move to back on center tape, get balls
    rsNavigateForward,    // move to fire position
    rsShoot,              // aim and fire
    rsNavigateBack,       // move to center tape
};

//robot states
extern RobotState robotState;

//initialization
void Takeoff(uint16 diff);

//running
void RunRobot(bool debug, RobotState debugState);

#endif