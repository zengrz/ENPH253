#include "menu_h.h"

long menuState;
int curSelection;
int knobValue;

int mUpdateDelay;
int mMenuDelay;

void InitializeMenu()
{
	knobValue = 0;
	curSelection = 1;
	menuState = 1;

	mUpdateDelay = 200;
	mMenuDelay = 500;
}

void UpdateMenu()
{
	if (startbutton())
	{
		ChangeMenuState();
		delay(mMenuDelay);
	}
	RefreshCurSelection();

	UpdateDisplay();

	delay(mUpdateDelay);
}

void UpdateDisplay()
{
	LCD.clear(); LCD.home();
	//  LCD.setCursor(6,1);
	//  LCD.print(knobValue);
	if (menuState == 1)
	{
		LCD.print("MAIN MENU");
		LCD.setCursor(0,1);
		LCD.print(curSelection);
		LCD.print(".");
		if (curSelection == 1)
		{
			LCD.print("RUN ROBOT");
		}
		else if (curSelection == 2)
		{
			LCD.print("DEBUG ROBOT");
		}
		else if (curSelection == 3)
		{
			LCD.print("TUNE ROBOT");
		}
		else if (curSelection == 8)
		{
			LCD.print("THIS PROG");
		}
	}
	else if (menuState == 11)
	{
		LCD.print("RUNNING...");
		LCD.setCursor(0,1);
		LCD.print(curSelection);
		LCD.print(".");
	}
	else if (menuState == 12)
	{
		LCD.print("DEBUG...");
		LCD.setCursor(0,1);
		LCD.print(curSelection);
		LCD.print(".");
		if (curSelection == 1)
		{
			LCD.print("DRIVE MOTOR");
		}
		else if (curSelection == 2)
		{
			LCD.print("SHOOT MOTOR");
		}
		else if (curSelection == 3)
		{
			LCD.print("ACQU MOTOR");
		}
		else if (curSelection == 4)
		{
			LCD.print("TARG SSR");
		}
		else if (curSelection == 5)
		{
			LCD.print("ACQU SSR");
		}
		else if (curSelection == 6)
		{
			LCD.print("GATE SSR");
		}
		else if (curSelection == 7)
		{
			LCD.print("PID CONTRLR");
		}
	}
	else if (menuState == 13)
	{
		LCD.print("TUNE...");
		LCD.setCursor(0,1);
		LCD.print(curSelection);
		LCD.print(".");
		if (curSelection == 1)
		{
			LCD.print("PID CONTRLR");
		}
		else if (curSelection == 2)
		{
			LCD.print("GATE SERVO");
		}
	}
	else if (menuState == 18)
	{
		LCD.print("");
		LCD.setCursor(0,1);
		LCD.print(curSelection);
		LCD.print(".");
		if (curSelection == 1)
		{
			LCD.print("UPDATE DELAY");
		}
		else if(curSelection == 2)
		{
			LCD.print("MENU DELAY");
		}
	}

	if (curSelection == 9)
	{
		LCD.setCursor(0,1);
		LCD.print(curSelection);
		LCD.print(".");
		LCD.print("BACK");
	}
}

void RefreshCurSelection()
{
	int readKnob = knob(6);
	if (abs(knobValue - readKnob) > 10)
	{
		if (readKnob < knobValue)
		{
			if (curSelection > 1) curSelection -= 1;
		}
		else
		{
			if (curSelection < 9) curSelection += 1; // poor condition, what if this level does not go all the way up to 9? sol: set 9 to be quit.
		}
		knobValue = readKnob;
	}
}

void ChangeMenuState()
{
	if (curSelection != 9)
	{
		DiveInOneLevel();
	}
	else
	{
		MoveBackOneLevel();
	}
	ResetMenuVars();
}

void DiveInOneLevel()
{
	menuState = menuState * 10 + curSelection;
	CallCorrespondingFunction();
}

void MoveBackOneLevel()
{
	menuState /= 10;  
}

void ResetMenuVars()
{
	curSelection = 1;
}

bool CallCorrespondingFunction()
{
	switch (menuState)
	{
	}
}