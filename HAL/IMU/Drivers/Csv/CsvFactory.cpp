#include <HAL/Devices/DriverFactory.h>
#include <HAL/Utils/StringUtils.h>

#include "CsvDriver.h"

namespace hal
{

class CsvFactory : public DriverFactory<IMUDriverInterface>
{
public:
    CsvFactory(const std::string& name)
        : DriverFactory<IMUDriverInterface>(name) {}

    std::shared_ptr<IMUDriverInterface> CreateDriver(const Uri& uri)
    {
        CsvDriver* pDriver = new CsvDriver( uri );
        return std::shared_ptr<IMUDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static CsvFactory g_CsvFactory("csv");

}
