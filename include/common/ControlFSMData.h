#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "common/DesiredCommand.h"
#include "common/LegController.h"
#include "common/Biped.h"
#include "common/StateEstimatorContainer.h"
#include "messages/LowLevelCmd.h"
#include "messages/LowlevelState.h"
#include "interface/IOInterface.h"

struct ControlFSMData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Biped *_biped;
    StateEstimatorContainer *_stateEstimator;
    LegController *_legController;
    DesiredStateCommand *_desiredStateCommand;
    IOInterface *_interface;
    LowlevelCmd *_lowCmd;
    LowlevelState *_lowState;

    void sendRecv()
    {
      _interface->sendRecv(_lowCmd, _lowState);
    }
};

#endif // CONTROLFSM_H