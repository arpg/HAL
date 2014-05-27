#include "NodeCarDriver.h"
#include <stdlib.h>
#include <unistd.h>
#include <HAL/Devices/DeviceException.h>
#include <PbMsgs/NodeCar.pb.h>

namespace hal {

/// CONSTRUCTOR

NodeCarDriver::NodeCarDriver(const Uri& uri) {
  debug_level_ = 0;
  device_id_ = uri.properties.Get<std::string>("id", "NodeCar");
  sim_name_ = uri.properties.Get<std::string>("sim", "LocalSim");
  device_name_ = uri.properties.Get<std::string>("name", device_id_);
  node_topic_ = device_name_;
  bool initDone = InitNode();
  initDone = RegisterInHost(uri);
  if (!initDone) {
    LOG(ERROR) << "Could not initialize the node or register the controller "
               << "in Simulator. The messages printed above should "
               << "be helpful.";
  } else {
    LOG(debug_level_) << "SUCCESS: NodeCar initialization";
  }
}

/// DESTRUCTOR

NodeCarDriver::~NodeCarDriver() {
}

///
/// INITIALIZE AND REGISTER
///

bool NodeCarDriver::InitNode() {
  node_.set_verbosity(0);
  LOG(debug_level_) << "Sim Name: " << sim_name_;
  LOG(debug_level_) << "Device Name: " << device_name_;
  if (node_.init(device_name_)==false) {
    LOG(ERROR) << "Cannot init NodeCar '" << device_name_;
    return false;
  }
  LOG(debug_level_) << "Device Topic: " << node_topic_;
  node_.advertise(node_topic_);
  return true;
}

bool NodeCarDriver::RegisterInHost(const Uri &uri) {
  pb::RegisterControllerReqMsg req;
  pb::RegisterControllerRepMsg rep;
  req.set_topic(node_topic_);
  req.set_uri(uri.ToString());
  int nTries = 0;
  while (nTries < 10000 &&
         node_.call_rpc(sim_name_, "RegisterControllerDevice",
                        req, rep) == false) {
    LOG(debug_level_) << "RPC call to register car controller failed";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    nTries++;
  }
  if(rep.success()) {
    LOG(debug_level_) << "Car controller Registered";
    return true;
  }
  else {
    LOG(ERROR) << "Car controller wasn't registered";
    return false;
  }
}

///
/// COMMANDS
///

bool NodeCarDriver::ApplyCommand(double dTorque, double dSteering) {
  pb::VehicleMsg msg;
  msg.set_steering_angle(dSteering);
  msg.set_desired_force(dTorque);
  msg.set_command_time(1.0/30.0);
  int nTries = 0;
  while (nTries < 5 && !node_.publish(node_topic_, msg)) {
    LOG(ERROR) << "Not able to publish commands";
    nTries++;
  }
  if (nTries<=5 && nTries!=0) {
    LOG(debug_level_) << "Publishing successfull after" << nTries << "tries.";
  }
  return true;
}

}
