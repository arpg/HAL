#include <stdio.h>
#include "AndCam.h"
#include <Node/Node.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Image.h>
#include <PbMsgs/NodeCamMessage.pb.h>
#include <HAL/Camera/CameraDevice.h>

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
  gCounter = 0;
  width = 640;
  height = 480;
  channels = 1;

  n.set_verbosity(9);
  n.init(string("LocalSim"));
  n.advertise(string("NodeCam"));
  n.provide_rpc(string("RegsiterCamDevice"), &RegsiterCamDevice, NULL);
}

void sendData( char* data, int w, int h, int format = pb::PB_LUMINANCE, int type = pb::PB_UNSIGNED_BYTE )
{
  gCounter++;
  pb::CameraMsg camMsg;
  pb::ImageMsg* pbImage = camMsg.add_image();
  char* temp = (char*) malloc (width*height);
  memcpy(temp, data, width*height);

  pbImage->set_height(height);
  pbImage->set_width(width);
  pbImage->set_format(pb::PB_LUMINANCE);
  pbImage->set_type(pb::PB_UNSIGNED_BYTE);
  pbImage->set_timestamp(gCounter);
  pbImage->set_data(temp);
  n.publish(topic, camMsg);
  free(temp);
}

int main( void )
{
  InitializeNode();
  hal::Camera cam("file:[loop=1]//~/Desktop/DataSets/LoopStereo/[Left*].pgm");
  width = cam.Width();
  height = cam.Height();
  shared_ptr<pb::ImageArray> pbIm = pb::ImageArray::Create();
  while (1) {
    cam.Capture(*pbIm);
    if (pbIm->Size() < 1) {
      cerr<<"There are no images here"<<endl;
      exit(1);
    }
    cout<<"About to send image"<<endl;
    sendData((char*) (pbIm->at(0)->data()), width, height, pb::PB_LUMINANCE, pb::PB_UNSIGNED_BYTE);
    usleep(1e5);
  }
  return 0;
}
