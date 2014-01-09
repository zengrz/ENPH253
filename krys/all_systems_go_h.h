#ifndef ALL_SYSTEMS_GO_H_
#define ALL_SYSTEMS_GO_H_

#include "utilities_h.h"
#include "pid_system_h.h"

void AllSystemsGo();

#define DEBUG_PAUSE                  StopMoving(); while (!startbutton()) {} while (startbutton()) {} LCD.print("(P)");

#define LCD_PRINTNEW(MSG)            LCD.clear(); LCD.home(); LCD.print(MSG)
#define LCD_PRINTBOT(MSG)            LCD.setCursor(0, 1); LCD.print(MSG)

#endif