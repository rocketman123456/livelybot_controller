#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSM/FSMState.h"

class FSMState_Passive: public FSMState
{
    public:
        FSMState_Passive(ControlFSMData *data);
        void enter();
        void run();
        void exit();
        FSMStateName checkTransition();
};

#endif