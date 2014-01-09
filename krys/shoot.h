#pragma once
#include "configuration_h.h"

enum ShooterState
{
    ssAim,
    ssFire,
    ssAlignBack,
};

enum AimState
{
    asCCW,
    asCW,
};

// state update (aim/fire)
void Shoot(uint16 diff);

// shooter control
void StartShooterControl();
void UpdateShooter(uint16 diff);
void StopShooterControl();
bool ShooterActivated();