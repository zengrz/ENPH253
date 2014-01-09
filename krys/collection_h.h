#ifndef COLLECTION_H_
#define COLLECTION_H_

#include "all_systems_go_h.h"

// pre: on center tape
// post: at the back, with at least 1 ball
void Collect(uint16 diff);
void MaintainCollectionSpeed(uint16 diff, uint16 motorCounts);

enum CollectionState
{
    csMoveBack = 0,
    csCollecting,
    csWaitForBall,
};

#endif