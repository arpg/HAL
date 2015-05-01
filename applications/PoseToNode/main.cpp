// Sends poses through a Node connection
//

#include <iostream>
#include <memory>
#include <chrono>

#include <gflags/gflags.h>

#include <HAL/Posys/PosysDevice.h>
#include <node/Node.h>
#include <HAL/Pose.pb.h>
#include <glog/logging.h>

DEFINE_string(nodeName, "posetonode", "Node name");
DEFINE_string(topic, "pose", "Topic name");
DEFINE_int32(verbosity, 2, "Verbosity level of Node "
                           "(the lower the less verbose)");
DEFINE_string(posys, "", "Posys driver");

//Global Node so that the callback has access to it
node::node n;
using std::cout;
using std::endl;

void Posys_Handler(hal::PoseMsg& PoseData) {
  /*
    cout << "Posys Id: " << PoseData.id() << ". Data: ";
    for (int ii = 0; ii < PoseData.pose().data_size(); ++ii) {
      cout << PoseData.pose().data(ii) << " ";
    }
    cout << endl;
  */
    n.publish(FLAGS_topic, PoseData);
    
}

int main(int argc, char *argv[])
{
  // get params
  google::SetUsageMessage("Read poses from a HAL driver and send them over "
                          "the network through Node. Runs as fast as possible.");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  
  if(FLAGS_posys.empty())
    {
      LOG(FATAL) << "Pose URI not provided. Use parameter -pose";
      return -1;
    }

  // initiate pose provider
  std::unique_ptr<hal::Posys> poseDevice;
  try {
    poseDevice.reset(new hal::Posys(FLAGS_posys));
  } catch (...) {
    LOG(FATAL) << "Could not create pose system from URI: " << FLAGS_posys;
  }

  // initiate node

  n.set_verbosity(FLAGS_verbosity);
  n.init(FLAGS_nodeName);
  n.advertise(FLAGS_topic);
  cout << "Node initialized" << endl;

  cout << "Publishing poses from node \"" << FLAGS_nodeName
       << "\" through topic \"" << FLAGS_topic << "\"" << endl;

  //The posys driver is implemented as a series of callbacks
  
  poseDevice->RegisterPosysDataCallback(&Posys_Handler);

  //Once the callback is registered, there's no further work to be done here

  while (true)
    sleep(1);

  return 0;
}
