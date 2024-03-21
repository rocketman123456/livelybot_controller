#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "messages/LowLevelCmd.h"
#include "messages/LowlevelState.h"
#include "interface/CmdPanel.h"

#include <string>

class IOInterface
{
    public:
        IOInterface(){}
        ~IOInterface(){}
        virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
        virtual float get_now_z() = 0;
        void zeroCmdPanel(){cmdPanel->setZero();} //secondary packaging
        void setPassive(){cmdPanel->setPassive();} //secondary packaging
        CmdPanel *cmdPanel;
};

#endif