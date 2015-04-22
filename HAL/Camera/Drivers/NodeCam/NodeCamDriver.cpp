#include <HAL/Devices/DeviceException.h>
#include "NodeCamDriver.h"

namespace hal
{

NodeCamDriver::NodeCamDriver(const std::string& sLocalNode,
                             const std::string& sRemoteNode,
                             const std::string& sTopicName,
                             double timeout)
  : m_sLocalNode(sLocalNode), m_sRemoteNode(sRemoteNode),
    m_sTopicName(sTopicName), m_dTimeout(timeout),
    m_sAddress(sRemoteNode + "/" + sTopicName),
    m_nChannels(0)
{
  InitNode();
}

NodeCamDriver::~NodeCamDriver()
{
}

bool NodeCamDriver::InitNode()
{
  m_Node.set_verbosity(2);
  if(m_Node.init(m_sLocalNode))
  {
    if(m_Node.subscribe(m_sAddress))
    {
      // this initiates the number of channels and image dimensions
      hal::CameraMsg vImages;
      Capture(vImages);
    }
    else
    {
      throw DeviceException("NodeCamDriver could not subscribe to \"" +
                            m_sAddress + "\"");
    }
  }
  else
  {
    throw DeviceException("NodeCamDriver could not initiatate node \"" +
                          m_sLocalNode + "\"");
  }
  return true;
}

bool NodeCamDriver::Capture(hal::CameraMsg& vImages)
{
  auto start = Now();
  bool ok = m_Node.receive(m_sAddress, vImages);
  for(; !ok && EllapsedSeconds(start, Now()) < m_dTimeout; )
  {
    ok = m_Node.receive(m_sAddress, vImages);
  }

  if(!ok)
  {
    throw DeviceException("NodeCamDriver error. Reception of images from \"" +
                          m_sAddress + "\" timed out");
  }

  m_nChannels = vImages.image_size();
  m_nImgWidth.resize(m_nChannels);
  m_nImgHeight.resize(m_nChannels);
  for(size_t i = 0; i < m_nChannels; ++i)
  {
    m_nImgWidth[i] = vImages.image(i).width();
    m_nImgHeight[i] = vImages.image(i).height();
  }

  return true;
}

std::string NodeCamDriver::GetDeviceProperty(const std::string&)
{
  return std::string();
}

size_t NodeCamDriver::NumChannels() const
{
  return  m_nChannels;
}

size_t NodeCamDriver::Width( size_t idx ) const
{
  return m_nImgWidth[idx];
}

size_t NodeCamDriver::Height( size_t idx ) const
{
  return m_nImgHeight[idx];
}

std::chrono::milliseconds NodeCamDriver::Now() const
{
  return std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::system_clock::now().time_since_epoch());
}

double NodeCamDriver::EllapsedSeconds(
    const std::chrono::milliseconds& begin,
    const std::chrono::milliseconds& end) const
{
  return static_cast<double>(end.count() - begin.count()) / 1e3;
}

}
