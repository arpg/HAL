#pragma once

#include <HAL/Car/CarDriverInterface.h>
#include <HAL/Utils/Uri.h>
#include <string>
#include <miniglog/logging.h>

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
  int debug_level_;
  node::node          node_;
  std::string         sim_name_;
  std::string         device_name_;
  std::string         device_id_;
  std::string         node_topic_;
};

}
