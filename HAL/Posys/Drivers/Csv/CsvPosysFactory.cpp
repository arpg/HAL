#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/StringUtils.h>
#include <HAL/Posys/PosysDriverInterface.h>

#include <string>
#include <sstream>
#include <iomanip>

#include "CsvPosysDriver.h"

namespace hal
{

class CsvPosysFactory : public DeviceFactory<PosysDriverInterface>
{
public:
    CsvPosysFactory(const std::string& name)
        : DeviceFactory<PosysDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<PosysDriverInterface> GetDevice(const Uri& uri)
    {
        const std::string sDataSourceDir = hal::ExpandTildePath(uri.url);
        const int nObjectId = uri.properties.Get("id", 0);

        std::stringstream filename;
        filename << sDataSourceDir;
        if (hal::IsDir(sDataSourceDir)) {
          filename << "/object" << std::setw(2) << std::setfill('0')
                   << nObjectId << ".csv";
        }

        CsvPosysDriver* pDriver = new CsvPosysDriver(filename.str());
        return std::shared_ptr<PosysDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static CsvPosysFactory g_CsvPosysFactory("csv");

}
