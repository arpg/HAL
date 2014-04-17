#pragma once

#include <HAL/Car/CarDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include <string>

//#pragma GCC system_header
#include <Node/Node.h>

namespace hal {

class NodeCarDriver : public CarDriverInterface {
 public:
  NodeCarDriver(const hal::Uri &uri);

  virtual ~NodeCarDriver();

  bool ApplyCommand(double dTorque, double dSteering) override;
  bool RegisterInHost(const Uri &uri);

  std::shared_ptr<CarDriverInterface> GetInputDevice() {
    return std::shared_ptr<CarDriverInterface>();
  }

  bool InitNode();

 private:
  node::node           m_Node;
  std::string         m_sSimNodeName;
  std::string         m_sDeviceName;
  std::string         m_sDeviceId;
  std::string         m_sTopic;
};

}
