#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "configuration_h.h"

enum NavigateBackState
{
	nbAlignBack,
	nbFindTape,
};

// pre: at back
// post: at a position to fire
void NavigateForward(uint16 diff);
// pre: aligned to back IR
// post: at center tape, facing back
void NavigateBackward(uint16 diff);

#endif