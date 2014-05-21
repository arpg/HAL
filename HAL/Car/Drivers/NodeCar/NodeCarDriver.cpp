#include "NodeCarDriver.h"
#include <stdlib.h>
#include <unistd.h>
#include <HAL/Devices/DeviceException.h>
#include <PbMsgs/NodeCar.pb.h>

/****************
 * Node name:
 ***************/


namespace hal {

/// CONSTRUCTOR

NodeCarDriver::NodeCarDriver(const Uri& uri) {
  device_id_ = uri.properties.Get<std::string>("id", "NodeCar");
  sim_name_ = uri.properties.Get<std::string>("sim", "LocalSim");
  device_name_ = uri.properties.Get<std::string>("name", device_id_);
  node_topic_ = device_name_;
  bool initDone = InitNode() && RegisterInHost(uri);
  if(!initDone) {
    throw DeviceException(
        "Could not initialize the node or register the controller "
        "in Simulator. The messages printed above should "
        "be helpful.");
  }
  std::cout<<"SUCCESS: NodeCar initialization"<<std::endl;
}

/// DESTRUCTOR

NodeCarDriver::~NodeCarDriver() {
}

///
/// INITIALIZE AND REGISTER
///

bool NodeCarDriver::InitNode() {
  node_.set_verbosity(2); // make some noise on errors
  std::cout<<"[NodDriver::InitNode] Sim Name: "<<sim_name_<<std::endl;
  std::cout<<"[NodDriver::InitNode] Device Name: "<<device_name_<<std::endl;
  if (node_.init(device_name_)==false) {
    std::cerr <<"[NodeCarDriver] Cannot init NodeCar '"<<device_name_<<"'"
              <<std::endl;
    return false;
  }
  std::cout<<"[NodeDriver::InitNode] Device Topic: "<<node_topic_<<std::endl;
  node_.advertise(node_topic_);
  return true;
}

bool NodeCarDriver::RegisterInHost(const Uri &uri) {
  pb::RegisterControllerReqMsg req;
  pb::RegisterControllerRepMsg rep;
  req.set_topic(node_topic_);
  req.set_uri(uri.ToString());
  int nTries = 0;
  while(nTries<1000000000 && node_.call_rpc(
          sim_name_, "RegisterControllerDevice", req, rep)==false){
    std::cerr << "[NodeCarDriver/RegisterInHost] RPC call to register car "
        "controller failed" << std::endl;
    usleep(1000000);
    nTries++;
  }
  if(rep.success()) {
    std::cout << "[NodeCarDriver/RegisterInHost] Car controller Registered"
              << std::endl;
    return true;
  }
  else {
    std::cerr << "[NodeCarDriver/RegisterInHost] Car controller wasn't "
                 "registered" << std::endl;
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
  int nTries = 0;
  while(nTries < 5 && !node_.publish(node_topic_, msg)) {
    std::cerr << "[NodeCarDriver/ApplyCommand()] Not able to publish commands"
              << std::endl;
    nTries++;
  }
  if(nTries<=5 && nTries!=0) {
    std::cerr<< "[NodeCarDriver/ApplyCommand()] Publishing successfull after"
             << nTries << "tries." << std::endl;
  }
  return true;
}

}
