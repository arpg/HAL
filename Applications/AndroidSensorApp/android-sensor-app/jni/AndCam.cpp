#include <stdio.h>
#include "AndCam.h"
#include <Node/Node.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Image.h>
#include <PbMsgs/NodeCamMessage.pb.h>
#include <HAL/Camera/CameraDevice.h>
#include <miniglog/logging.h>

using namespace std;

void RegsiterCamDevice(RegisterNodeCamReqMsg& req, RegisterNodeCamRepMsg& rep, void* UserData)
{
  rep.set_regsiter_flag(1);
  rep.set_time_step(gCounter);
  rep.set_width(width);
  rep.set_height(height);
  rep.set_channels(channels);
}

bool InitializeNode( void )
{
//  __android_log_print(ANDROID_LOG_VERBOSE, "Native!", "about to initialize");
  ANDROID_LOG_VERBOSE << "Native!";
  string NodeName("LocalSim");
  string Topic("NodeCam");
  n.set_verbosity(9);
  n.init(NodeName);
  n.advertise(Topic);
  n.provide_rpc(std::string("RegsiterCamDevice"), &RegsiterCamDevice, NULL);
}

void sendData( unsigned char* data, int width, int height, int format = pb::PB_RGB, int type = pb::PB_UNSIGNED_BYTE )
{
  std::shared_ptr<pb::ImageArray> vImages = pb::ImageArray::Create();
  gCounter++;
  pb::CameraMsg camMsg;
  pb::ImageMsg* pbImage = camMsg.add_image();
  pbImage->set_height(height);
  pbImage->set_width(width);
  pbImage->set_format(static_cast<pb::Format>(format));
  pbImage->set_type(static_cast<pb::Type>(type));
  pbImage->set_timestamp(gCounter);
  pbImage->set_data((const char*) data);
  n.publish(topic, camMsg);
}
