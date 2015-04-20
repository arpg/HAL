#pragma once

#include <HAL/Posys/PosysDriverInterface.h>

#include <Messages/Reader.h>

namespace hal {

class MicroStrainPosysDriver : public PosysDriverInterface
{
public:
    MicroStrainPosysDriver();
    ~MicroStrainPosysDriver();
    void RegisterPosysDataCallback(PosysDriverDataCallback callback);
  bool IsRunning() const override {
    return true;
  }
};

} /* namespace */
