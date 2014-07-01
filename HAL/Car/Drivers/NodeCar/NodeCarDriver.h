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
  bool InitNode();
  bool RegisterInHost(const Uri &uri);
  bool ApplyCommand(double torque, double steering,
                    double command_time) override;


 private:
  int debug_level_;
  node::node          node_;
  std::string         sim_name_;
  std::string         device_name_;
  std::string         device_id_;
  std::string         node_topic_;
};

}
