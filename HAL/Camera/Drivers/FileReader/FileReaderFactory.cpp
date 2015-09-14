#include <HAL/Devices/DriverFactory.h>
#include "FileReaderDriver.h"

namespace hal {

  class FileReaderFactory : public DriverFactory<CameraDriverInterface> {
    public:
      FileReaderFactory(const std::string& name)
        : DriverFactory<CameraDriverInterface>(name) { }

      std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri) 
      {
        FileReaderDriver* filereader = new FileReaderDriver( uri );
        return std::shared_ptr<CameraDriverInterface>(filereader);
      }
  };

  // Register this factory by creating static instance of factory
  static FileReaderFactory g_FileReaderFactory("file");
  static FileReaderFactory g_FileReaderFactoryS("files");

}  // namespace hal
