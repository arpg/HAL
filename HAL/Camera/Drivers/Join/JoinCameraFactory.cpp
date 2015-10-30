#include <HAL/Devices/DeviceFactory.h>
#include "JoinCameraDriver.h"

namespace hal
{

class JoinCameraFactory : public DeviceFactory<CameraDriverInterface>
{
public:
  JoinCameraFactory(const std::string& name)
    : DeviceFactory<CameraDriverInterface>(name)
  {
    Params() = {};
  }

  std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
  {
    std::vector<Uri> suburis = splitUri(uri.url);
    std::vector<std::shared_ptr<CameraDriverInterface>> cameras;
    cameras.reserve(suburis.size());

    for( const Uri& uri : suburis ) {
      try {
        std::cout << "Creating stream from uri: " << uri.ToString()
                  << std::endl;
        cameras.emplace_back
            (DeviceRegistry<hal::CameraDriverInterface>::Instance().Create(uri));
      } catch ( std::exception& e ) {
        throw DeviceException(std::string("Error creating driver from uri \"") +
                              uri.ToString() + "\": " + e.what());
      }
    }

    if( cameras.empty() ) {
      throw DeviceException("No input cameras given to join");
    }

    JoinCameraDriver* pDriver = new JoinCameraDriver(cameras);
    return std::shared_ptr<CameraDriverInterface>( pDriver );
  }

  std::vector<Uri> splitUri(const std::string& url)
  {
    const char C = '&'; // split token

    std::vector<Uri> ret;
    std::string::size_type begin = 0, end = 0;
    for(; end != std::string::npos; begin = end + 1 ) {
      end = url.find( C, begin );
      std::string s;
      if( end == std::string::npos )
        s = url.substr(begin);
      else
        s = url.substr(begin, end - begin);
      if( !s.empty() ) ret.emplace_back( Uri(s) );
    }
    return ret;
  }

};

// Register this factory by creating static instance of factory
static JoinCameraFactory g_JoinCameraFactory("join");

}
