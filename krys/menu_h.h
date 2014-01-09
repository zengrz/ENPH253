#ifndef MENU_H_
#define MENU_H_

#include <phys253.h>
#include <LiquidCrystal.h>

void InitializeMenu();
void UpdateMenu();
void UpdateDisplay();
void RefreshCurSelection();
void ChangeMenuState();
void DiveInOneLevel();
void MoveBackOneLevel();
void ResetMenuVars();
bool CallCorrespondingFunction();

#endif