#include <HAL/Devices/DriverFactory.h>
#include "FileReaderDriver.h"

namespace hal
{
  class FileReaderFactory : public DriverFactory<CameraDriverInterface> {
    public:

      std::shared_ptr<CameraDriverInterface> CreateDriver(const Uri& uri) 
      {
        FileReaderDriver* filereader = new FileReaderDriver( default_params_, uri );
        return std::shared_ptr<CameraDriverInterface>(filereader);
      }
  };

  // Register this factory by creating static instance of factory
  static FileReaderFactory g_FileReaderFactory("file",  
      {
      {"startframe", "0", "First frame to capture."},
      {"loop", "false", "Play beginning once finished."},
      {"grey", "false", "Convert internally to greyscale."},
      {"buffer", "10", "Number of frames to cache in memory"},
      {"frequency", "30", "Capture frequency to emulate if needed"},
      {"name", "FileCam", "Camera name."},
      {"id", "0", "Camera id (serial number or UUID)."}
      } );

  static FileReaderFactory g_FileReaderFactoryS("files",
      {
      {"startframe", "0", "First frame to capture."},
      {"loop", "false", "Play beginning once finished."},
      {"grey", "false", "Convert internally to greyscale."},
      {"buffer", "10", "Number of frames to cache in memory"},
      {"frequency", "30", "Capture frequency to emulate if needed"},
      {"name", "FileCam", "Camera name."},
      {"id", "0", "Camera id (serial number or UUID)."}
      } );
}

