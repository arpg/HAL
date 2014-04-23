#include <stdio.h>
#include "AndCam.h"
#include <Node/Node.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Image.h>
#include <PbMsgs/NodeCamMessage.pb.h>
#include <HAL/Camera/CameraDevice.h>
#include <android/Log.h>

using namespace std;

void RegsiterCamDevice(RegisterNodeCamReqMsg& req, RegisterNodeCamRepMsg& rep, void* UserData)
{
  rep.set_regsiter_flag(1);
  rep.set_time_step(gCounter);
  rep.set_width(width);
  rep.set_height(height);
  rep.set_channels(channels);
}

bool InitializeNode( string IP, int port )
{
  gCounter = 0;
  width = 640;
  height = 480;
  channels = 1;

  n.set_verbosity(-2);
  n.init(string("LocalSim"));
  n.advertise(string("NodeCam"));
  n.provide_rpc(string("RegsiterCamDevice"), &RegsiterCamDevice, NULL);

  msg::GetTableResponse rep;
  n.ConnectNode( IP, port, &rep);
}

void sendData( char* data, int w, int h, int format = pb::PB_LUMINANCE, int type = pb::PB_UNSIGNED_BYTE )
{
  if (data != NULL) {
    gCounter++;
    pb::CameraMsg camMsg;
    pb::ImageMsg* pbImage = camMsg.add_image();
    pbImage->set_height(height);
    pbImage->set_width(width);
    pbImage->set_format(pb::PB_LUMINANCE);
    pbImage->set_type(pb::PB_UNSIGNED_BYTE);
    pbImage->set_timestamp(gCounter);
    pbImage->set_data(data, width*height);
    n.publish(topic, camMsg);
  }
}
