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

using namespace pangolin;


bool        g_bLog      = false;
pb::Logger& g_Logger    = pb::Logger::GetInstance();

pangolin::DataLog g_PlotLogAccel;
pangolin::DataLog g_PlotLogGryo;
pangolin::DataLog g_PlotLogMag;

//Camera Variables
bool bCamsRunning = false;
std::vector<std::shared_ptr<pb::ImageArray> > vImgArr;
std::vector<std::vector<pb::Image> > vvImgs;
std::vector<hal::Camera> vCams;

//For lidar
void ConvertRangeToPose(pb::LidarMsg& LidarData, VelodyneCalib* vc);
float *ptp;//[9216000];
unsigned char *col;//[9216000];
pangolin::GlBuffer *buf;
pangolin::GlBuffer *colBuf;
unsigned char colMap[6000];//2000(2.0m) * 3 (rgb)
pb::Velodyne vld("/Users/jongnarr/Codes/CoreDev/HAL/Applications/LexusLogger/db.xml");

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

  if( g_bLog ) {
    pb::Msg pbMsg;
    pbMsg.set_timestamp( hal::Tic() );
    pbMsg.mutable_pose()->Swap(&PoseData);
    g_Logger.LogMessage(pbMsg);
  }
}

void LIDAR_Handler(pb::LidarMsg& LidarData)
{
  //static bool first = true;
  //static struct VelodyneCalib vc[64];
  //if(first)
  //{
  //  ReadCalib("/home/rpg/Code/CoreDev/HAL/Applications/SensorViewer/db.xml", vc);
  //  first = false;
  //}

  vld.ConvertRangeToPoints(LidarData);
  ptp = vld.getPoints();
  col = vld.getCol();
  static double prev_clear_time = hal::Tic();
  if(LidarData.system_time() - prev_clear_time > 2)
  {
    memset(ptp, 0, sizeof(ptp));
    memset(col, 0, sizeof(col));
    prev_clear_time = LidarData.system_time();
  }
  //ConvertRangeToPose(LidarData, vc);
  //buf->Upload(ptp, sizeof(ptp));//possible but would require restructuring, seg fault right now.
  if(g_bLog)
  {
    pb::Msg pbMsg;
    pbMsg.set_timestamp(hal::Tic());
    pbMsg.mutable_lidar()->Swap(&LidarData);
    g_Logger.LogMessage(pbMsg);
  }
}

void ConvertRangeToPose(pb::LidarMsg& LidarData, VelodyneCalib* vc)
{
  //block contains upper and lower block, i.e. 64 lasers.
  for(int block=0; block<6; block++)
  {
    //calculating sine and cos beforehand, to save computation later
    double cos_rotation_pos=cos(LidarData.rotational_position().data(block)*M_PI/180);
    double sin_rotation_pos=sin(LidarData.rotational_position().data(block)*M_PI/180);

    for(int laser=0; laser<64;laser++)
    {
      //printf("--------------------------------------------------------------------\n");
      int pt_idx = block*64+laser;

      //if the distance is 0, that means it was either max range or less tha 0.9, so invalid, in that case we don't do anything
      double distance = LidarData.distance().data(pt_idx);
      double distance_raw = distance;
      if(distance==0)
      {
        continue;
      }

      //getting a pointer to calibration data for this laser.
      VelodyneCalib vcl = vc[laser];//vcl stands for Velodyne calibration of a laser.

      /* 1. Correct the distance, that is done by just adding the value with distCorrection (which is far point calibration at 25.04m).
       *    This is the distance error along the ray which a laser has.
       *    Distance from LidarMsg is already converted in meters, so was the correction factor in calibration data.
       **/
      double dist_corr = vcl.distCorrection;
      distance += dist_corr;
      //printf("pt_id = %d\ndist=%.3lf, dist_corr=%.1lf, dist_cor = %.1lf\n", pt_idx, distance, dist_corr, vc[laser].distCorrection);
      //printf("cos_rot_ang = %.1lf, sin = %.1lf\n", cos_rotation_pos, sin_rotation_pos);

      /* 2. Now we correct angles, these angles are with the front of the camera, which is +y.
       *    These values will be primarily used when we will be computing x and y coordinate.
       *    If a is angle of laser, b is correction, to correct angle we want a-b, but finally for calculationswe want cos(a-b), we have cos of a,b.
       *    So we use the identity cos(a-b) = cos(a)cos(b) + sin(a)*sin(b)
       *    Similarly for sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
       **/
      double cos_rotation_angle = cos_rotation_pos*vcl.cos_rotCorrection + sin_rotation_pos*vcl.sin_rotCorrection;
      double sin_rotation_angle = sin_rotation_pos*vcl.cos_rotCorrection - vcl.sin_rotCorrection*cos_rotation_pos;
      //printf("cos_rot_corr= %.1lf, sin = %.1lf\n", vcl.cos_rotCorrection, vcl.sin_rotCorrection);


      /* 3. Now we compute the distance in xy plane, i.e. the distance in horizontal plane and not along the ray.
       *    This is done to correct vertical and horizontal offset for the laser. Each laser is supposed to originate from a single point,
       *    which obviously doesn't happen. Vertical Offset is the offset along z-axis from xy-plane, a positive offset is towards +z.
       *    Horizontal offset is the offset in xy plane from the origin, a +ve offset is towards -x.
       *    To get the final results, some more adjustments are made in next step.
       **/
      double cos_vert_corr = vcl.cos_vertCorrection;
      double sin_vert_corr = vcl.sin_vertCorrection;
      double horiz_offset = vcl.horizOffsetCorrection;
      double vert_offset = vcl.vertOffsetCorrection;
      //now variable names are dist in corresponding planes or axis
      double xy = distance * cos_vert_corr;
      double xx = xy * sin_rotation_angle  - horiz_offset * cos_rotation_angle;
      double yy = xy * cos_rotation_angle  + horiz_offset * sin_rotation_angle;
      xx = xx<0?-xx:xx;
      yy = yy<0?-yy:yy;
      //printf("cos_vert_corr= %.1lf, sin = %.1lf\n", vcl.cos_vertCorrection, vcl.sin_vertCorrection);
      //printf("vert_off = %.1lf, horiz_off = %.1lf\n", vert_offset, horiz_offset);
      //printf("xx=%.1lf, yy=%.1lf\n", xx, yy);

      /* 4. Now, we correct for parameters distCorrectionX and distCorrectionY. Why we do this is unclear, but; what we know is what we do.
       *    The idea here is that we have correction value for near points at x=2.4m and y=1.93m, we also have distCorrection for far point
       *    calibration at 25.04m. So, to get the correction for a particular distance in x, we interpolate using values corresponding to x-axis
       *    i.e. 2.4, distCorrectionX, 25.04, distCorrection (last two apply to both cases). Similarly for y-axis using 1.93,distCorrectionY,
       *    25.04, distCorrection.
       *    For better understanding of the interpolation formulae refer to Appendix F of the manual.
       **/
      //compute the correction via interpolation
      double corr_xx = vcl.distCorrectionX + (dist_corr - vcl.distCorrectionX) * (xx-2.4)/22.64;//25.04-2.4 = 22.64
      double corr_yy = vcl.distCorrectionY + (dist_corr - vcl.distCorrectionY) * (yy-1.93)/23.11;//25.04-1.93 = 23.11

      /* 5. Next task is to extract coordinates x, y and z.
       *    To compute x and y cordinates we correct distance with corrections computed, then take projection on xy palne, then on respective axes.
       *    Ofcourse we correct for horizontal offset as well, we are not stupid you know.
       *    z is a good boy and calculating it is straight forward, projection of distance on z-axis and then correcting by vertOffset.
       **/
      //If B=distance in the follwing three formulae, then To B or Not to B is the question.
      //The X-coordinate
      xy = (distance_raw + corr_xx)*cos_vert_corr;
      //points[pt_idx][0]=  xy * sin_rotation_angle  - horiz_offset * cos_rotation_angle;
      xx =  xy * sin_rotation_angle  - horiz_offset * cos_rotation_angle;
      //x.push_back(xx);

      //The Y-coordinate
      xy = (distance_raw + corr_yy)*cos_vert_corr;
      //points[pt_idx][1] =  xy * cos_rotation_angle  + horiz_offset * sin_rotation_angle;
      yy =  xy * cos_rotation_angle  + horiz_offset * sin_rotation_angle;
      //y.push_back(yy);

      //The Z-coordinate
      //points[pt_idx][2] = distance * sin_vert_corr + vert_offset;
      double zz=distance_raw * sin_vert_corr + vert_offset + 1.5;//velodyne is porbably at 1.5m from ground, adding 1.5 to have the ground plane at z=0, exact value to be estimated later.

      //we have angular resolution of 0.01 degrees. For each rotational position (there are 36000 such positions) we have a block of 256 (64(laser) * 4(x,y,z,1) = 256) float values.
      //Angle supplied by LidarData is in degrees, so to compute a rotational position we multiply angle by 100, then we multiply by 256 to reach the offset for that block, hence
      //multiplication by 25600.
      //Adding each laser gives us data worth of 4 floats, so we multiply laser by 4 to get exact position in array.
      int idx = ((int)LidarData.rotational_position().data(block))*25600 + laser*4;//
      ptp[idx] = (float)xx;
      ptp[idx+1] = (float)yy;
      ptp[idx+2] = (float)zz;
      ptp[idx+3] = 1.0;

      //color the point
      int cidx = floor(zz*1000)*3;//m to mm. 3 is for three bytes of r,g,b.
      col[idx] = colMap[cidx];
      col[idx+1] = colMap[cidx+1];
      col[idx+2] = colMap[cidx+2];
      col[idx+3] = 255;
    }
  }

}

void Camera_Handler(int id) {
  bool bRun = true;

  while(bRun) {
    // Captures the image, if capture fails bRun=false
    // o.w. log data in case logging enabled.
    //std::cout<<"id="<<id<<std::flush;
    if(bCamsRunning){
      //std::shared_ptr<pb::ImageArray> img = pb::ImageArray::Create();
      bool success = vCams[id].Capture(*vImgArr[id]);


      if(!success) {
        bRun = false;
      }
      else {
        if(g_bLog ) {
          pb::Msg pbMsg;
          pbMsg.set_timestamp( hal::Tic() );
          pbMsg.mutable_camera()->Swap(&vImgArr[id]->Ref());
          g_Logger.LogMessage(pbMsg);
        }
      }
    }
    usleep(100000);
  }
  std::cout<<"End"<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{

  GetPot clArgs(argc,argv);
  double dCamViewTop = 1, dImuViewBottom = 0.1, dLidarViewBottom = 0.1;
  double dImuViewRight = 1, dLidarViewLeft = 0;

  ///-------------------- CAMERA INIT (Optional)

  std::vector<std::string> vsCamsUri;
  vsCamsUri.push_back(clArgs.follow("", "-cam"));
  std::string sCamUri;
  do {
    sCamUri = clArgs.follow("","-cam");
    //std::cout<<sCamUri<<std::endl;
    vsCamsUri.push_back(sCamUri);
  } while(vsCamsUri[0]!=vsCamsUri[vsCamsUri.size()-1]);
  vsCamsUri.pop_back();

  const bool bHaveCam= !vsCamsUri.empty();

  std::vector<std::thread> vCamThreads;
  int nNumCam = 0;
  if(bHaveCam) {
    nNumCam = vsCamsUri.size();
    for(int ii=0;ii<nNumCam; ii++) {
      std::cout<<"arg = "<<vsCamsUri[ii]<<std::endl;
      vCams.push_back(hal::Camera(vsCamsUri[ii]));
      vImgArr.push_back(pb::ImageArray::Create());
      vCamThreads.push_back( std::thread( Camera_Handler, ii) );
    }
    vvImgs.resize(nNumCam);

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
    //build_jet_map(2000, colMap);//to cover 2m in z;
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
  pangolin::CreatePanel("ui").SetBounds(0,0.1,0,1);
  pangolin::View& cameraView = pangolin::Display("Camera");
  //cameraView.SetLayout(pangolin::LayoutEqualHorizontal);
  cameraView.SetLayout(pangolin::LayoutEqual);
  cameraView.SetBounds(0.1,dCamViewTop,0.0,1.0);
  pangolin::DisplayBase().AddDisplay(cameraView);
  if(bHaveCam)
  {
    for(int ii=0; ii < nNumCam; ++ii ) {
      std::cout<<"img["<< ii<<"] w = "<<vCams[ii].Width()<<std::endl;
      cameraView.AddDisplay(pangolin::CreateDisplay()
                            .SetAspect(
                              vCams[ii].Width() / (double)vCams[ii].Height()
                              ));
    }
  }

  if( bHaveIMU ) {
    pangolin::View& imuView = pangolin::CreateDisplay();
    imuView.SetLayout(pangolin::LayoutEqualVertical);
    imuView.SetBounds(dImuViewBottom,1,0,dImuViewRight);
    imuView.AddDisplay( pangolin::CreatePlotter("Accel", &g_PlotLogAccel));
    imuView.AddDisplay( pangolin::CreatePlotter("Gryo", &g_PlotLogGryo));
    imuView.AddDisplay( pangolin::CreatePlotter("Mag", &g_PlotLogMag));
    pangolin::DisplayBase().AddDisplay(imuView);
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

  bCamsRunning = true;//starting cameras just before starting GUI.
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
        //std::cout<< "img SIZE = "<< vImgArr[ii]->Size()<<std::endl;
        if(vImgArr[ii]->Size() == 0)
          continue;
        pb::Image img = vImgArr[ii]->at(0);
        //Only initialise now we know format.
        GLenum imgFormat = img.Format();
        GLenum imgType = img.Type();

        if(!glTex[ii].tid) {
          pangolin::GlTexture gltex;
          glTex[ii].Reinitialise(
                vCams[ii].Width(), vCams[ii].Height(), imgFormat,
                true, 0, imgFormat , imgType, 0
                );
        }

        cameraView[ii].Activate();
        glTex[ii].Upload(img.data(),
                         imgFormat, imgType);
        glTex[ii].RenderToViewportFlipY();
      }
    }

    //Draw circle if logging has started.
    if(g_bLog) {
      // draw red circle on bottom left corner for visual cue
      if( ! ((nFrame / 30) %2) ) {
        cameraView[0].ActivatePixelOrthographic();
        pangolin::GlState state;
        state.glDisable(GL_DEPTH_TEST);
        state.glDisable(GL_LIGHTING);
        glColor3f( 1.0, 0, 0 );
        pangolin::glDrawCircle(20,20,7);
      }
    }

    if(bHaveLIDAR)
    {
      buf->Upload(ptp, sizeof(ptp));
      colBuf->Upload(col, sizeof(col));
    }

    pangolin::FinishFrame();
  }

  bCamsRunning = false;
  for (int ii = 0; ii < nNumCam; ++ii)
  {
    if(vCamThreads[ii].joinable())
      vCamThreads[ii].join();
  }
  return 0;
}
