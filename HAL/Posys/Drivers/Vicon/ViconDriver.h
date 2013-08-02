#pragma once

#include <HAL/Posys/PosysDriverInterface.h>

namespace hal {

class ViconDriver : public PosysDriverInterface
{
public:
    ViconDriver();
    ~ViconDriver();
    void RegisterPosysDataCallback(PosysDriverDataCallback callback);

private:
    PosysDriverDataCallback     m_Callback;

};

} /* namespace */
