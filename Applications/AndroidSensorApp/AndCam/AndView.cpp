//#include <jni.h>
#include <stdio.h>
#include "AndCam.h"
#include <Node/Node.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Image.h>
#include <HAL/Camera/CameraDevice.h>

using namespace std;

int main( int argc, char** argv )
{
  string sNodeName = "OtherNode";
  int nUpTime      = 1000;
  int nVerbosity   = 0;

  // initialize node
  node::node n;
  n.set_verbosity(nVerbosity); // be a bit noisy
  if( n.init(sNodeName) == false ){
      return -1;
  }

  pb::CameraMsg camMsg;
  std::string topic("LocalSim/NodeCam");
  n.subscribe(topic);
  //    pbImage->set_width();
  //    pbImage->set_height();
  //    pbImage->set_format();
  //    pbImage->set_type();

  std::string sSpinner = "|/-\\";
  double dt = 0.2;
  int ii = 0;
  for( double t = 0.0; t < nUpTime; t+=dt ){
      cout << "\b\b" << sSpinner[ii++%4] << flush;
      n.receive(topic, camMsg);
      cout << "Message received " << camMsg.ByteSize() << endl;
//      for (int i = 0; i < 10; i++ ) {
//        cout << (int) (unsigned char) camMsg.image(0).data()[i] <<"\t";
//      }
//      cout<<endl;
      usleep(1e6);
  }

  printf( "'%s' Exiting cleanly\n", sNodeName.c_str() );

  return 0;
}
