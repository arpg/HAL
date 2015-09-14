#include <HAL/Devices/DriverFactory.h>
#include <HAL/Utils/StringUtils.h>
#include <HAL/Posys/PosysDriverInterface.h>

#include <string>
#include <sstream>
#include <iomanip>

#include "CsvPosysDriver.h"

namespace hal
{

class CsvPosysFactory : public DriverFactory<PosysDriverInterface>
{
public:
    CsvPosysFactory(const std::string& name)
        : DriverFactory<PosysDriverInterface>(name) {}

    std::shared_ptr<PosysDriverInterface> CreateDriver(const Uri& uri)
    {
        const std::string sDataSourceDir = hal::ExpandTildePath(uri.url);
        const int nObjectId = uri.properties.GetProperty("id", 0);

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
