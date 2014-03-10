#include "NodeCarDriver.h"
#include <stdlib.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/PbMsgs/NodeCar.pb.h>

namespace hal
{

NodeCarDriver::NodeCarDriver(const Uri& uri) {
  m_sDeviceId = uri.properties.Get<std::string>("id", "NodeCar");
  m_sSimNodeName = uri.properties.Get<std::string>("sim", "LocalSim");

  //Default value is device Id as id will be used when name is not present.
  m_sDeviceName = uri.properties.Get<std::string>("name", m_sDeviceId);

  // Initialize and create the node, if the node is initialized then register
  // the camera in simba.
  m_sTopic = m_sDeviceName + "/Command";
  bool initDone = InitNode() && RegisterInHost(uri);
  if(!initDone) {
    throw DeviceException("Could Not initialize the node or register the "
                          "camera in Simulator, messages printed above should"
                          " be helpful.");
  }

  std::cout<<"init NodeCar success"<<std::endl;
}


NodeCarDriver::~NodeCarDriver()
{
}

bool NodeCarDriver::ApplyCommand(double dTorque, double dSteering) {
  pb::VehicleMsg msg;
  msg.set_steering_angle(dSteering);
  msg.set_desired_force(dTorque);

  int nTries=0;
  while(nTries<5 &&
        !m_Node.publish(m_sTopic, msg)) {
    std::cerr << "[NodeCarDriver/ApplyCommand()] Not able to publish commands"
              << std::endl;
    nTries++;
  }
  if(nTries==5) {
    std::cerr<< "[NodeCarDriver/ApplyCommand()] Command wasn't published after "
                "5 tries" << std::endl;
    return false;
  }
  else if(nTries>0) {
    std::cerr<< "[NodeCarDriver/ApplyCommand()] Publishing successfull after"
             << nTries << "tries." << std::endl;
  }

  return true;
}

void NodeCarDriver::RegisterInHost() {
  pb::RegisterControllerReqMsg req;
  pb::RegisterControllerRepMsg rep;

  req.set_topic(m_sTopic);
  int nTries=0;
  while(nTries<5 &&
        !m_Node.call_rpc(m_sSimNodeName, "RegisterControllerDevice", req, rep))
  {
    std::cerr << "[NodeCarDriver/RegisterInHost()] RPC call to register car "
                 "controller failed" << std::endl;
    sleep(0.2);
    nTries++;
  }

  if(rep.success()) {
    std::cout << "[NodeCarDriver/RegisterInHost()] Car controller Registered"
              << std::endl;
    return true;
  }
  else {
    std::cerr << "[NodeCarDriver/RegisterInHost()] Car controller wasn't "
                 "registered" << std::endl;
    return false;
  }
}

bool NodeCarDriver::InitNode() {
  m_Node.set_verbosity(2); // make some noise on errors
  if(m_Node.init(m_sDeviceName)==false)
  {
    std::cerr <<"[NodeCarDriver] Cannot init NodeCar '"<<m_sDeviceName<<"'"
              <<std::endl;
    return false;
  }

  m_Node.advertise(m_sTopic);

  return true;
}

}
