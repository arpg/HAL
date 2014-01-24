#include <pangolin/pangolin.h>
#include <pangolin/timer.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Posys/PosysDevice.h>

#include <HAL/Utils/GetPot>
#include <HAL/Utils/TicToc.h>

#include <PbMsgs/ImageArray.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Matrix.h>

#include <SceneGraph/SceneGraph.h>

bool        g_bLog      = false;
pb::Logger& g_Logger    = pb::Logger::GetInstance();

using std::placeholders::_1;

class SensorViewer {
 public:
  SensorViewer() : num_channels_(0), base_width_(0), base_height_(0),
                   has_camera_(false), has_imu_(false), has_posys_(false),
                   is_running_(true), is_stepping_(false), frame_number_(0),
                   panel_width_(0),
                   logger_(pb::Logger::GetInstance()) {
#ifdef ANDROID
    logger_.LogToFile("/sdcard/", "sensors");
#else
    logger_.LogToFile("", "sensors");
#endif
  }
  virtual ~SensorViewer() {}

  void SetupGUI() {
    panel_width_ = base_width_ / 6;
    int window_width = num_channels_ * base_width_ + panel_width_;
    pangolin::OpenGlRenderState render_state;
    pangolin::Handler3D handler(render_state);
    pangolin::CreateWindowAndBind("SensorViewer",
                                  window_width,
                                  base_height_ * (has_imu_ ? 3.0/2.0 : 1.0));

    glPixelStorei(GL_PACK_ALIGNMENT,1);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);

    // Create Smart viewports for each camera image that preserve aspect
    pangolin::View& cameraView = pangolin::Display("camera").
        SetLayout(pangolin::LayoutEqual);
    for (size_t ii = 0; ii < num_channels_; ++ii) {
      float right = panel_width_ + (ii + 1) * base_width_;
      cameraView.AddDisplay(pangolin::CreateDisplay().
                            SetAspect((double)base_width_ / base_height_).
                            SetBounds(1.0, 0.0,
                                      pangolin::Attach::Pix(panel_width_),
                                      pangolin::Attach::Pix(right), true));
    }
    pangolin::CreatePanel("ui").SetBounds(1.0, 0.0, 0,
                                          pangolin::Attach::Pix(panel_width_));
    logging_enabled_.reset(new pangolin::Var<bool>("ui.Enable logging",
                                                   false));

    if (has_imu_) {
      pangolin::View& imuView = pangolin::CreateDisplay().
          SetLayout(pangolin::LayoutEqualVertical);
      imuView.AddDisplay(pangolin::CreatePlotter("Accel", &g_PlotLogAccel));
      imuView.AddDisplay(pangolin::CreatePlotter("Gryo", &g_PlotLogGryo));
      imuView.AddDisplay(pangolin::CreatePlotter("Mag", &g_PlotLogMag));

      if (has_camera_) {
        cameraView.SetBounds(1.0/3.0, 1.0, 0.0, 1.0);
        imuView.SetBounds(0, 1.0/3.0, 0.0, 1.0);
      }
    }    
}
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

    pb::VectorMsg pbVec = PoseData.pose();
    if( PoseData.id() == 0 ) {
//        printf("%4f   %4f   %4f -- %4f   %4f   %4f   %4f\r",pbVec.data(0),pbVec.data(1),pbVec.data(2),
//           pbVec.data(3),pbVec.data(4),pbVec.data(5),pbVec.data(6));
//        fflush(stdout);
//        glAxis.SetPose( pbVec.data(0),pbVec.data(1),pbVec.data(2), pbVec.data(3),pbVec.data(4),pbVec.data(5),pbVec.data(6) );
        g_Pose(0) = pbVec.data(0);
        g_Pose(1) = pbVec.data(1);
        g_Pose(2) = pbVec.data(2);
        g_Pose(3) = pbVec.data(3);
        g_Pose(4) = pbVec.data(4);
        g_Pose(5) = pbVec.data(5);
        g_Pose(6) = pbVec.data(6);
    }

  }

  void Run() {
    RegisterCallbacks();

    pangolin::GlTexture glTex[num_channels_];
    pangolin::View& cameraView = pangolin::Display("camera");

    pangolin::Timer timer;
    bool got_first_image = false;
    bool capture_success = false;
    std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();
    for (; !pangolin::ShouldQuit(); ++frame_number_) {
      const bool go = is_running_ || pangolin::Pushed(is_stepping_);
      usleep(20000);

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glColor4f(1.0f,1.0f,1.0f,1.0f);

      if (go && num_channels_) {
        capture_success = camera_.Capture(*images);
        if (!got_first_image && capture_success) {
          got_first_image = true;
        }

#ifdef HAVE_GLUT
        if (frame_number_ % 30 == 0) {
          char buffer[1024];
          sprintf(buffer,"SensorViewer (FPS: %f)", 30.0 / timer.Elapsed_s());
          glutSetWindowTitle(buffer);
          timer.Reset();
        }
#endif
#if ANDROID
        if (frame_number_ % 30 == 0) {
          LOGI("SensorViewer (FPS: %f)", 30.0 / timer.Elapsed_s());
          timer.Reset();
        }
#endif
      }

      if (got_first_image) {
        for (size_t ii = 0; ii < num_channels_; ++ii) {
          pb::Image img = images->at(ii);
          if (!glTex[ii].tid && num_channels_) {
            GLint internal_format = (img.Format() == GL_LUMINANCE ?
                                     GL_LUMINANCE : GL_RGBA);
            // Only initialise now we know format.
            glTex[ii].Reinitialise(img.Width(), img.Height(),
                                   internal_format, true, 0,
                                   img.Format(), img.Type(), 0);
          }

          cameraView[ii].Activate();
          if (got_first_image && img.data()) {
            glTex[ii].Upload(img.data(), img.Format(), img.Type());
            glTex[ii].RenderToViewportFlipY();
          }
        }
      }

      if (*logging_enabled_ && is_running_) {
        if (capture_success) {
          LogCamera(images.get());
        }
        DrawLoggingIndicator();
      }

      pangolin::FinishFrame();
    }
  }

  void set_camera(const std::string& cam_uri) {
    try {
      camera_ = hal::Camera(cam_uri);
    } catch (const hal::DeviceException& e) {
      std::cerr << "Camera failed to open!" << std::endl;
      abort();
    }
    has_camera_ = true;

    num_channels_ = camera_.NumChannels();
    base_width_ = camera_.Width();
    base_height_ = camera_.Height();

    for (size_t ii=0; ii < num_channels_; ++ii) {
      std::cout << "\t" << camera_.Width(ii) << "x"
                << camera_.Height(ii) << std::endl;
    }
  }

  void set_imu(const std::string& imu_uri) {
    imu_ = hal::IMU(imu_uri);
    has_imu_ = true;
  }

  void set_posys(const std::string& posys_uri) {
    posys_ = hal::Posys(posys_uri);
    has_posys_ = true;
  }

 protected:
  void RegisterCallbacks() {
    if (has_posys_) {
      posys_.RegisterPosysDataCallback(std::bind(&SensorViewer::Posys_Handler,
                                                 this, _1));
      std::cout << "- Registering Posys device." << std::endl;
    }
  pangolin::View& imuView = pangolin::CreateDisplay().SetLayout(pangolin::LayoutEqualVertical);

  if( bHaveIMU ) {
    imuView.AddDisplay( pangolin::CreatePlotter("Accel", &g_PlotLogAccel) );
    imuView.AddDisplay( pangolin::CreatePlotter("Gryo", &g_PlotLogGryo) );
    imuView.AddDisplay( pangolin::CreatePlotter("Mag", &g_PlotLogMag) );
>>>>>>> Stashed changes

    if (has_imu_) {
      imu_.RegisterIMUDataCallback(
          std::bind(&SensorViewer::IMU_Handler, this, _1));
      std::cout << "- Registering IMU device." << std::endl;
    }
  }

<<<<<<< Updated upstream
  /// Draw red circle on bottom left corner for visual cue
  void DrawLoggingIndicator() {
    if (frame_number_ % 60 < 20) {

      pangolin::Display("camera").ActivatePixelOrthographic();
      pangolin::GlState state;
      state.glDisable(GL_DEPTH_TEST);
      state.glDisable(GL_LIGHTING);
      glColor3f(1.0, 0, 0);
      pangolin::glDrawCircle(panel_width_ + 20, 20, 7);
    }
  }
=======
  SceneGraph::GLSceneGraph glGraph;
  SceneGraph::GLAxis glAxis;
  glGraph.AddChild( &glAxis );

  if (bHavePosys) {

      pangolin::View& posysView = pangolin::CreateDisplay();
      const double far = 10*1000;
      const double near = 1E-3;

      pangolin::OpenGlRenderState stacks3d(
          pangolin::ProjectionMatrix(1280,480,420,420,320,240,near,far),
                  pangolin::ModelViewLookAt(0, 0, -2, 0, 0, 1, pangolin::AxisNegY)
      );

      if (bHaveCam && bHaveIMU) {
          cameraView.SetBounds(1.0/3.0, 1.0, 0.0, 0.5);
          imuView.SetBounds(0, 1.0/3.0, 0.0, 1.0);
          posysView.SetBounds(0, 1.0/3.0, 0.5, 1.0);
      }
      else if (bHaveCam) {
          cameraView.SetBounds(0, 1.0, 0.0, 0.5);
          posysView.SetBounds(0, 1.0, 0.5, 1.0);
      }
      else if (bHaveIMU) {
          posysView.SetBounds(1.0/3.0, 1.0, 0.0, 1.0);
          imuView.SetBounds(0, 1.0/3.0, 0.0, 1.0);
      }

      posysView.SetHandler(new SceneGraph::HandlerSceneGraph(glGraph,stacks3d))
            .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glGraph, stacks3d));

//      posysView.AddDisplay( pangolin::CreateDisplay() );
  }

  bool bRun = true;
  bool bStep = false;
  unsigned long nFrame=0;

  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&bStep](){bStep=true;} );
  pangolin::RegisterKeyPressCallback(' ', [&](){bRun = !bRun;} );
  pangolin::RegisterKeyPressCallback('l', [&](){ g_bLog = !g_bLog; nFrame = 0; } );

  pangolin::Timer theTimer;

  for(; !pangolin::ShouldQuit(); nFrame++)
  {
    const bool bGo = bRun || pangolin::Pushed(bStep);

#ifdef HAVE_GLUT
      if(nFrame%30 == 0) {
        char buffer[1024];
        sprintf(buffer,"SensorViewer (FPS: %f)", 30.0 / theTimer.Elapsed_s() );
        glutSetWindowTitle(buffer);
        theTimer.Reset();
      }
#endif
#if ANDROID
      if(nFrame%30 == 0) {
        LOGI("SensorViewer (FPS: %f)", 30.0 / theTimer.Elapsed_s());
        theTimer.Reset();
      }
#endif

    if (IMUdata.has_accel()) {
      g_PlotLogAccel.Log(IMUdata.accel().data(0),
                         IMUdata.accel().data(1),
                         IMUdata.accel().data(2));
    }
    if (IMUdata.has_gyro()) {
      g_PlotLogGryo.Log(IMUdata.gyro().data(0),
                        IMUdata.gyro().data(1),
                        IMUdata.gyro().data(2));
    }
    if (IMUdata.has_mag()) {
      g_PlotLogMag.Log(IMUdata.mag().data(0),
                       IMUdata.mag().data(1),
                       IMUdata.mag().data(2));
    }
  }

  void Posys_Handler(pb::PoseMsg& PoseData) {
    if (logging_enabled_) {
      pb::Msg pbMsg;
      pbMsg.set_timestamp(hal::Tic());
      pbMsg.mutable_pose()->Swap(&PoseData);
      logger_.LogMessage(pbMsg);
    }
  }

 private:
  size_t num_channels_, base_width_, base_height_;
  bool has_camera_, has_imu_, has_posys_;
  bool is_running_, is_stepping_;
  int frame_number_;
  int panel_width_;
  hal::Camera camera_;
  hal::IMU imu_;
  hal::Posys posys_;
  std::unique_ptr<pangolin::Var<bool> > logging_enabled_;
  pb::Logger& logger_;
};

int main(int argc, char* argv[]) {
  GetPot cl_args(argc, argv);

  std::string cam_uri = cl_args.follow("", "-cam");
  std::string imu_uri = cl_args.follow("", "-imu");
  std::string posys_uri = cl_args.follow("", "-posys");

#ifdef ANDROID
  if (cam_uri.empty()) {
    cam_uri = "kitkat://";
  }
#endif

  SensorViewer viewer;
  if (!cam_uri.empty()) {
    viewer.set_camera(cam_uri);
  }

  if (!imu_uri.empty()) {
    viewer.set_imu(imu_uri);
  }

  if (!posys_uri.empty()) {
    viewer.set_camera(posys_uri);
  }

  viewer.SetupGUI();
  viewer.Run();
}
