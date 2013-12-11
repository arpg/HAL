#pragma once

#include <HAL/Posys/PosysDriverInterface.h>

#include <PbMsgs/Reader.h>

namespace hal {

class MicroStrainPosysDriver : public PosysDriverInterface
{
public:
    MicroStrainPosysDriver();
    ~MicroStrainPosysDriver();
    void RegisterPosysDataCallback(PosysDriverDataCallback callback);
};

} /* namespace */
