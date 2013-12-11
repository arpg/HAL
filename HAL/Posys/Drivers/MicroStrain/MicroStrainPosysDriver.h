#pragma once

#include <HAL/Posys/PosysDriverInterface.h>

#include <PbMsgs/Reader.h>

namespace hal {

class MicroStrainPosysDriver : public PosysDriverInterface
{
public:
    MicroStrainPosysDriver(std::string filename);
    ~MicroStrainPosysDriver();
    void RegisterPosysDataCallback(PosysDriverDataCallback callback);

private:
    PosysDriverDataCallback     m_callback;
};

} /* namespace */
