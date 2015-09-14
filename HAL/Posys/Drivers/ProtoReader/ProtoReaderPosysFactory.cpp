#include <HAL/Devices/DriverFactory.h>

#include "ProtoReaderPosysDriver.h"

namespace hal
{

class ProtoReaderPosysFactory : public DriverFactory<PosysDriverInterface>
{
public:
    ProtoReaderPosysFactory(const std::string& name)
        : DriverFactory<PosysDriverInterface>(name)
    {}

    std::shared_ptr<PosysDriverInterface> CreateDriver(const Uri& uri)
    {
        const std::string file = ExpandTildePath(uri.url);

        ProtoReaderPosysDriver* pDriver = new ProtoReaderPosysDriver(file);
        return std::shared_ptr<PosysDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static ProtoReaderPosysFactory g_ProtoReaderPosysFactory1("proto");
static ProtoReaderPosysFactory g_ProtoReaderPosysFactory2("log");

}
