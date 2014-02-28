#include <pangolin/timer.h>
#include <pangolin/pangolin.h>
#include <pangolin/glvbo.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Posys/PosysDevice.h>
#include <HAL/LIDAR/LIDARDevice.h>

#include <HAL/Utils/GetPot>
#include <HAL/Utils/TicToc.h>

#include <PbMsgs/Logger.h>
#include <PbMsgs/Matrix.h>

/*SceneGraph, to use containers and stuff */
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/PangolinGlCachedSizeableBuffer.h>

/* Helper headers for Velodyne */
#include <tinyxml2.h>//From Library tinyxml2, duh
#include "CalibParser.h"//is in this folder itself
#include <math.h>//Standard header, but only used in velodyne.
#include <vector>//To capture points in it.
//#include <mvl/image/colormap.h>
#include <Velodyne.h>

/* Generic Includes */
#include <queue>

using namespace pangolin;


bool        g_bLog      = false;
pb::Logger& g_Logger    = pb::Logger::GetInstance();
std::mutex logMutex;

pangolin::DataLog g_PlotLogAccel;
pangolin::DataLog g_PlotLogGryo;
pangolin::DataLog g_PlotLogMag;

// Camera Variables
bool bCamsRunning = false;
std::vector<std::shared_ptr<pb::ImageArray> > vImgCap;
std::vector<hal::Camera> vCams;
std::deque<pb::Image> vImgs;
// should be vector of mutex, but vector requires object to be movable, but
// apparently mutex isn't, which makes sense. deque doesn't require movable
// objects, hence is used. Also initialization is complicated, push_back and
// such don't work, but a simple resize does. :)
std::deque <std::mutex> vCamMtx;

// For lidar
void ConvertRangeToPose(pb::LidarMsg& LidarData, VelodyneCalib* vc);
float *ptp;//[9216000];
unsigned char *col;//[9216000];
pangolin::GlBuffer *buf;
pangolin::GlBuffer *colBuf;
unsigned char colMap[6000];//2000(2.0m) * 3 (rgb)
pb::Velodyne vld("/home/rpg/Code/CoreDev/HAL/Applications/LexusLogger/db.xml");

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Handler(pb::ImuMsg& IMUdata)
{
  if( g_bLog ) {
    pb::Msg pbMsg;
    pbMsg.set_timestamp( hal::Tic() );
    pbMsg.mutable_imu()->Swap(&IMUdata);
    g_Logger.LogMessage(pbMsg);
  }
  //    const pb::VectorMsg& pbVec = IMUdata.accel();
  //    printf("X: %5f    Y: %5f     Z: %5f\r",pbVec.data(0),pbVec.data(1),pbVec.data(2));
  if( IMUdata.has_accel() ) {
    g_PlotLogAccel.Log( IMUdata.accel().data(0), IMUdata.accel().data(1), IMUdata.accel().data(2) );
  }
  if( IMUdata.has_gyro() ) {
    g_PlotLogGryo.Log( IMUdata.gyro().data(0), IMUdata.gyro().data(1), IMUdata.gyro().data(2) );
  }
  if( IMUdata.has_mag() ) {
    g_PlotLogMag.Log( IMUdata.mag().data(0), IMUdata.mag().data(1), IMUdata.mag().data(2) );
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Posys_Handler(pb::PoseMsg& PoseData)
{
  /*
    pb::VectorMsg pbVec = PoseData.pose();
    if( PoseData.id() == 0 ) {
        printf("%4f   %4f   %4f -- %4f   %4f   %4f   %4f\r",pbVec.data(0),pbVec.data(1),pbVec.data(2),
           pbVec.data(3),pbVec.data(4),pbVec.data(5),pbVec.data(6));
        fflush(stdout);
    }
    */

  std::cout<<"lat: "<<PoseData.pose().data(0)<<", long: "
           <<PoseData.pose().data(1) <<", height: "
           <<PoseData.pose().data(2) <<std::endl;
  if( g_bLog ) {
    pb::Msg pbMsg;
    pbMsg.set_timestamp( hal::Tic() );
    pbMsg.mutable_pose()->Swap(&PoseData);
    g_Logger.LogMessage(pbMsg);
  }
}

void LIDAR_Handler(pb::LidarMsg& LidarData)
{

  static double prev_clear_time = hal::Tic();
  if(LidarData.system_time() - prev_clear_time > 2)
  {
    memset(ptp, 0, vld.getPointsSize()*4);
    memset(col, 0, vld.getPointsSize());
    prev_clear_time = LidarData.system_time();
  }

  vld.ConvertRangeToPoints(LidarData);
  ptp = vld.getPoints();
  col = vld.getCol();

  if(g_bLog)
  {
    pb::Msg pbMsg;
    pbMsg.set_timestamp(hal::Tic());
    pbMsg.mutable_lidar()->Swap(&LidarData);
    g_Logger.LogMessage(pbMsg);
  }
}

void Camera_Handler(int id) {
  bool bRun = true;
  bool bStarted = false;

  while(bRun) {
    // Captures the image, if capture fails bRun=false
    // o.w. log data in case logging enabled.
    if(bCamsRunning){
      //std::cout<<"id="<<id<<std::flush;
      vCamMtx[id].lock();
      bool success = vCams[id].Capture(*vImgCap[id]);
//      vImgs[id] = *(vImgCap[id]->at(0));
      vCamMtx[id].unlock();

      if(!success) {
        bRun = false;
      }
      else {
	if(!bStarted)
	    bStarted=true;
        if(g_bLog ) {
          pb::Msg pbMsg;
          logMutex.lock();
          pbMsg.set_timestamp( hal::Tic() );
//          pbMsg.mutable_camera()->Swap(&vImgCap[id]->Ref());
          pbMsg.mutable_camera()->CopyFrom(vImgCap[id]->Ref());
          g_Logger.LogMessage(pbMsg);
          logMutex.unlock();
        }
      }
    }
    else if(bStarted){
	bRun=false;
    }
    usleep(3000);
  }
  std::cout<<"Camera "<< id << "stopped." <<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{

  GetPot clArgs(argc,argv);
  double dCamViewTop = 1, dImuViewBottom = 0, dLidarViewBottom = 0;
  double dImuViewRight = 1, dLidarViewLeft = 0;

  ///-------------------- CAMERA INIT (Optional)

  std::vector<std::string> vsCamsUri;
  vsCamsUri.push_back(clArgs.follow("", "-cam"));
  std::string sCamUri;
  do {
    sCamUri = clArgs.follow("","-cam");
    std::cout<<sCamUri<<std::endl;
    vsCamsUri.push_back(sCamUri);
  } while(vsCamsUri[0]!=vsCamsUri[vsCamsUri.size()-1]);
  vsCamsUri.pop_back();

  const bool bHaveCam= (vsCamsUri[0]!=""); //if no cam it will have 1 element "".

  std::vector<std::thread> vCamThreads;
  int nNumCam = 0;
  if(bHaveCam) {
    nNumCam = vsCamsUri.size();
    for(int ii=0;ii<nNumCam; ii++) {
      std::cout<<"arg = "<<vsCamsUri[ii]<<std::endl;
      vCams.push_back(hal::Camera(vsCamsUri[ii]));
      vImgCap.push_back(pb::ImageArray::Create());
      vCamThreads.push_back( std::thread( Camera_Handler, ii) );
      //pb::Image img(vImgCap[ii]->Ref().image(0), vImgCap[ii]);
      //vImgs.push_back(img);
    }
    vCamMtx.resize(nNumCam);
    //if IMU and Lidar are present they would have to make space for camera.
    dImuViewBottom = 0.5; dLidarViewBottom = 0.5;
  }

  ///-------------------- IMU INIT (Optional)

  std::string sIMU = clArgs.follow("", "-imu" );
  const bool bHaveIMU = !sIMU.empty();

  hal::IMU theIMU;
  if( bHaveIMU ) {
    theIMU = hal::IMU(sIMU);
    theIMU.RegisterIMUDataCallback(IMU_Handler);
    std::cout << "- Registering IMU device." << std::endl;

    dCamViewTop = 0.5; dLidarViewLeft = 1/3;
  }


  ///-------------------- POSYS INIT (Optional)

  std::string sPosys = clArgs.follow("", "-posys" );
  const bool bHavePosys = !sPosys.empty();

  hal::Posys thePosys;
  if( bHavePosys ) {
    thePosys = hal::Posys(sPosys);
    thePosys.RegisterPosysDataCallback(Posys_Handler);
    std::cout << "- Registering Posys device." << std::endl;
  }

  ///-------------------- LIDAR INIT (Optional)

  std::string sLIDAR = clArgs.follow("", "-lidar" );
  const bool bHaveLIDAR = !sLIDAR.empty();

  hal::LIDAR theLIDAR;
  if( bHaveLIDAR ) {
    theLIDAR = hal::LIDAR(sLIDAR);
    theLIDAR.RegisterLIDARDataCallback(LIDAR_Handler);
    std::cout << "- Registering LIDAR device." << std::endl;

    dCamViewTop = 0.5; dImuViewRight = 1/3;
  }


  ///-------------------- WINDOW INIT
  int nWindowHeight=768, nWindowWidth=1024;

  // Setup OpenGL Display (based on GLUT)
  pangolin::CreateWindowAndBind("Logging the Lexus, Yo!!",nWindowWidth,nWindowHeight);

  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  //Declarations for lidar
  SceneGraph::GLSceneGraph glgraph;
  SceneGraph::GLGrid glgrid(50, 2.0, true);
  glgraph.AddChild(&glgrid);

  buf = new pangolin::GlBuffer(pangolin::GlArrayBuffer, 2304000, GL_FLOAT, 4, GL_DYNAMIC_DRAW);
  colBuf = new pangolin::GlBuffer(pangolin::GlArrayBuffer, 2304000, GL_UNSIGNED_BYTE, 4, GL_DYNAMIC_DRAW);
  SceneGraph::GLVbo glvbo(buf,0,colBuf);
  glgraph.AddChild(&glvbo);


  // Create Smart viewports for each camera image that preserve aspect
//  pangolin::CreatePanel("ui").SetBounds(0,0.1,0,1);
  pangolin::View& cameraView = pangolin::Display("Camera");
  //cameraView.SetLayout(pangolin::LayoutEqualHorizontal);
  cameraView.SetLayout(pangolin::LayoutEqual);
  cameraView.SetBounds(0.1,dCamViewTop,0.0,1.0);
  pangolin::DisplayBase().AddDisplay(cameraView);
  if(bHaveCam)
  {
    for(int ii=0; ii < nNumCam; ++ii ) {
      cameraView.AddDisplay(pangolin::CreateDisplay()
                            .SetAspect(
                              vCams[ii].Width() / (double)vCams[ii].Height()
                              ));
    }
  }

  if( bHaveIMU ) {
    pangolin::View& imuView = pangolin::CreateDisplay();
    imuView.SetLayout(pangolin::LayoutEqualVertical);
    imuView.AddDisplay( pangolin::CreatePlotter("Accel", &g_PlotLogAccel));
    imuView.AddDisplay( pangolin::CreatePlotter("Gryo", &g_PlotLogGryo));
    imuView.AddDisplay( pangolin::CreatePlotter("Mag", &g_PlotLogMag));
    pangolin::DisplayBase().AddDisplay(imuView);
    imuView.SetBounds(dImuViewBottom,1,0,dImuViewRight);
  }

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
        ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
        ModelViewLookAt(0,-12,4, 0,1,0, AxisZ)
        );
  pangolin::View& d_cam = pangolin::CreateDisplay();

  if(bHaveLIDAR)//Add the 3d view only if LIDAR is to be seen
  {
    d_cam.SetBounds(dLidarViewBottom, 1, dLidarViewLeft, 1)
        .SetHandler(new Handler3D(s_cam))
        .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glgraph, s_cam));
    pangolin::DisplayBase().AddDisplay(d_cam);
  }

  bool bRun = true;
  bool bStep = false;
  unsigned long nFrame=0;

  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&bStep](){bStep=true;} );
  pangolin::RegisterKeyPressCallback(' ', [&](){bRun = !bRun;} );
  pangolin::RegisterKeyPressCallback('l', [&](){ g_bLog = !g_bLog; nFrame = 0; } );

  pangolin::Timer theTimer;

  bCamsRunning = bHaveCam;//starting cameras just before starting GUI.
  usleep(100);
  pangolin::GlTexture glTex[nNumCam];
  for(; !pangolin::ShouldQuit(); nFrame++)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    if(bRun && bCamsRunning) {

#ifdef HAVE_GLUT
      if(nFrame%30 == 0) {
        char buffer[1024];
        sprintf(buffer,"SensorViewer (FPS: %f)", 30.0 / theTimer.Elapsed_s() );
        glutSetWindowTitle(buffer);
        theTimer.Reset();
      }
#endif

      for(int ii=0; ii<nNumCam; ++ii ) {

        if(vImgCap[ii]->Size() == 0 )
          continue;
        vCamMtx[ii].lock();
        pb::Image img = *(vImgCap[ii]->at(0));
//	pb::Image img = vImgs[ii];
        GLenum imgFormat = img.Format();
        GLenum imgType = img.Type();

        //Only initialise now we know format.
        if(!glTex[ii].tid) {
          glTex[ii].Reinitialise(
                vCams[ii].Width(), vCams[ii].Height(), imgFormat,
                true, 0, imgFormat , imgType, 0
                );
        }
        cameraView[ii].Activate();
        glTex[ii].Upload(img.data(),
                         imgFormat,
                         imgType);
        glTex[ii].RenderToViewportFlipY();
        vCamMtx[ii].unlock();
      }
    }

    //Draw circle if logging has started.
    if(g_bLog) {
      // draw red circle on bottom left corner for visual cue
      if( ! ((nFrame / 30) %2) ) {
        pangolin::DisplayBase().ActivatePixelOrthographic();
        pangolin::GlState state;
        state.glDisable(GL_DEPTH_TEST);
        state.glDisable(GL_LIGHTING);
        glColor3f( 1.0, 0, 0 );
        pangolin::glDrawCircle(20,20,7);
      }
    }

    if(bHaveLIDAR)
    {
      buf->Upload(ptp, vld.getPointsSize()*4);
      colBuf->Upload(col, vld.getPointsSize());
    }

    pangolin::FinishFrame();
    usleep(10000);
  }

  bCamsRunning = false;
  for (int ii = 0; ii < nNumCam; ++ii)
  {
    if(vCamThreads[ii].joinable())
      vCamThreads[ii].join();
  }
  return 0;
}
