#include <pangolin/timer.h>
#include <pangolin/pangolin.h>
#include <pangolin/glvbo.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Posys/PosysDevice.h>
#include <HAL/LIDAR/LIDARDevice.h>
#include <HAL/Encoder/EncoderDevice.h>

#include <HAL/Utils/GetPot>
#include <HAL/Utils/TicToc.h>

#include <PbMsgs/Logger.h>
#include <PbMsgs/Matrix.h>

/* related to OSM tiling */
#include "gui-engine/include/pangolin_tile_view.h"

/*SceneGraph, to use containers and stuff */
#include <SceneGraph/SceneGraph.h>
#include <SceneGraph/GLMesh.h>
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

pangolin::DataLog g_PlotCarAccel;
pangolin::DataLog g_PlotCarGyro;

// Camera Variables
bool bCamsRunning = false;
std::vector<std::shared_ptr<pb::ImageArray> > vImgCap;
std::vector<hal::Camera> vCams;
std::vector<std::shared_ptr<pb::Image> > vImgs;
// should be vector of mutex, but vector requires object to be movable, but
// apparently mutex isn't, which makes sense. deque doesn't require movable
// objects, hence is used. Also initialization is complicated, push_back and
// such don't work, but a simple resize does. :)
std::deque <std::mutex> vCamMtx;

// For lidar
float *ptp;//[9216000];
unsigned char *col;//[9216000];
pangolin::GlBuffer *buf;
pangolin::GlBuffer *colBuf;
pb::Velodyne vld;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Handler(pb::ImuMsg& IMUdata)
{
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
  if( g_bLog ) {
    pb::Msg pbMsg;
    pbMsg.set_timestamp( hal::Tic() );
    pbMsg.mutable_imu()->Swap(&IMUdata);
    g_Logger.LogMessage(pbMsg);
  }
  //    const pb::VectorMsg& pbVec = IMUdata.accel();
}

void CarIMU_Handler(pb::ImuMsg& IMUdata)
{
  if( IMUdata.has_accel() ) {
    g_PlotCarAccel.Log( IMUdata.accel().data(0), IMUdata.accel().data(1));
  }
  if( IMUdata.has_gyro() ) {
    g_PlotCarGyro.Log( IMUdata.gyro().data(0) );
  }
  if( g_bLog ) {
    pb::Msg pbMsg;
    pbMsg.set_timestamp( hal::Tic() );
    pbMsg.mutable_imu()->Swap(&IMUdata);
    g_Logger.LogMessage(pbMsg);
  }
}

void Encoder_Handler(pb::EncoderMsg &EncoderData)
{
  for (int ii = 0; ii < EncoderData.label_size(); ++ii)
  {
    if(EncoderData.label(ii) == "ENC_RATE_FL") {
      std::cout<<"Encoder FL = "<<EncoderData.data(ii)<<std::endl;
    }
    else if(EncoderData.label(ii) == "ENC_RATE_FR") {
      std::cout<<"Encoder FR = "<<EncoderData.data(ii)<<std::endl;
    }
    else if(EncoderData.label(ii) == "ENC_RATE_RL") {
      std::cout<<"Encoder RL = "<<EncoderData.data(ii)<<std::endl;

    }
    else if(EncoderData.label(ii) == "ENC_RATE_RR") {
      std::cout<<"Encoder RR = "<<EncoderData.data(ii)<<std::endl;

    }
    else if(EncoderData.label(ii) == "RAW_STEERING_DATA") {
      std::cout<<"Steering = "<<EncoderData.data(ii)<<std::endl;
    }
  }
  if( g_bLog ) {
    pb::Msg pbMsg;
    pbMsg.set_timestamp( hal::Tic() );
    pbMsg.mutable_encoder()->Swap(&EncoderData);
    g_Logger.LogMessage(pbMsg);
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
      std::shared_ptr<pb::ImageArray> temp = pb::ImageArray::Create();
      bool success = vCams[id].Capture(*temp);

      if(!success) {
        //bRun = false;
      }
      else {
	if(!bStarted)
	    bStarted=true;
        vCamMtx[id].lock();
        vImgCap[id] = temp;
        vCamMtx[id].unlock();
        if(g_bLog ) {
          pb::Msg pbMsg;
          logMutex.lock();
          pbMsg.set_timestamp( hal::Tic() );
          vImgCap[id]->Ref().set_id(id+1);
          pbMsg.mutable_camera()->CopyFrom(vImgCap[id]->Ref());
          g_Logger.LogMessage(pbMsg);
          logMutex.unlock();
        }
      }
    }
    else if(bStarted){
	bRun=false;
    }
    usleep(1000);
  }
  std::cout<<"Camera "<< id << "stopped." <<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{

  GetPot clArgs(argc,argv);

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
    vImgs.resize(nNumCam);
    //if IMU and Lidar are present they would have to make space for camera.
  }

  ///-------------------- IMU INIT (Optional)

  std::vector<std::string> sIMU; sIMU.reserve(2);
  sIMU.push_back( clArgs.follow("", "-imu" ));
  const bool bHaveIMU = !sIMU[0].empty();

  std::vector<hal::IMU> theIMU;
  if( bHaveIMU ) {
  sIMU.push_back(clArgs.follow("", "-imu"));
  if(sIMU[0].compare(sIMU[1]) ==0)
    sIMU.pop_back();
  for (int ii = 0; ii < sIMU.size(); ++ii)
  {
    theIMU.push_back( hal::IMU(sIMU[ii]));
    size_t bPcanPos = sIMU[ii].find("pcan");
    if(bPcanPos == std::string::npos) {
      theIMU[ii].RegisterIMUDataCallback(IMU_Handler);
      std::cout << "- Registering IMU device." << std::endl;
    }
    else {
      theIMU[ii].RegisterIMUDataCallback(CarIMU_Handler);
      std::cout << "- Registering Car IMU device." << std::endl;
    }
  }

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
    vld.Init("/home/rpg/Code/CoreDev/HAL/Applications/LexusLogger/db.xml");
    theLIDAR = hal::LIDAR(sLIDAR);
    theLIDAR.RegisterLIDARDataCallback(LIDAR_Handler);
    std::cout << "- Registering LIDAR device." << std::endl;

  }

  ///-------------------- ENCODER INIT (Optional)
  std::string sEncoder = clArgs.follow("","-encoder");
  const bool bHaveEncoder = !sEncoder.empty();

  hal::Encoder carEncoder;
  if(bHaveEncoder) {
    carEncoder = hal::Encoder(sEncoder);
    carEncoder.RegisterEncoderDataCallback(Encoder_Handler);
    std::cout << "- Registering Encoder device." << std::endl;
  }
  ///-------------------- WINDOW INIT
  int nWindowHeight=768, nWindowWidth=1024;
  double dCamViewBottom = 0.8, dCamViewLeft=0.6, dImuViewBottom = 0.6, dLidarViewTop = 1.0;
  double dImuViewLeft = 0.6, dLidarViewRight = 0.6, dImuViewTop = 0.8;

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

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
        ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
        ModelViewLookAt(0,-12,4, 0,1,0, AxisZ)
        );
  pangolin::View& d_cam = pangolin::CreateDisplay();

  SceneGraph::GLMesh carMesh;
  if(bHaveLIDAR)//Add the 3d view only if LIDAR is to be seen
  {
    d_cam.SetBounds(0.0, dLidarViewTop, 0.0, dLidarViewRight)
        .SetHandler(new Handler3D(s_cam))
        .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glgraph, s_cam));
    d_cam.SetAspect(-640.0/480.0);
    pangolin::DisplayBase().AddDisplay(d_cam);
    carMesh.Init("/home/rpg/Downloads/02/3ds/02_lowpoly.3DS");
    carMesh.SetPerceptable(true);
    carMesh.SetScale(0.01);
    carMesh.SetPose(0, 0, 0, M_PI/2, 0, M_PI);
    carMesh.SetAlpha(0.4);
    glgraph.AddChild(&carMesh);
  }


  // Create Smart viewports for each camera image that preserve aspect
  pangolin::View& cameraView = pangolin::Display("Camera");

  pangolin::DisplayBase().AddDisplay(cameraView);
  if(bHaveCam)
  {
    for(int ii=0; ii < nNumCam; ++ii ) {
      cameraView.AddDisplay(pangolin::CreateDisplay()
                            .SetAspect(
                              vCams[ii].Width() / (double)vCams[ii].Height()
                              ));
    }
    cameraView.SetLayout(pangolin::LayoutEqualHorizontal);
    cameraView.SetBounds(dCamViewBottom, 1.0,dCamViewLeft,1.0);
  }

  if( bHaveIMU ) {
    for (int ii = 0; ii < sIMU.size(); ++ii)
    {
      size_t bPcanPos = sIMU[ii].find("pcan");
      pangolin::View& imuView = pangolin::CreateDisplay();
        imuView.SetLayout(pangolin::LayoutEqual);
      if(bPcanPos == std::string::npos) {
        imuView.AddDisplay( pangolin::CreatePlotter("Accel", &g_PlotLogAccel));
        imuView.AddDisplay( pangolin::CreatePlotter("Gryo", &g_PlotLogGryo));
      }
      else {
        imuView.AddDisplay(pangolin::CreatePlotter("CarAccel", &g_PlotCarAccel));
        imuView.AddDisplay(pangolin::CreatePlotter("CarGyro", &g_PlotCarGyro));
      }
      imuView.SetBounds(dImuViewBottom, dImuViewTop,dImuViewLeft,1.0);
    }
  }

  // if ( bHavePosys ) {
    // pangolin::TileView tileView;
    // pangolin::DisplayBase().AddDisplay(tileView);
    // tileView.SetBounds(0.0,0.6,0.8,1.0);
    // tileView.SetZoom(17);
    // tileView.AddGPSPoint(38.903973,-77.048918);
  //}

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
