#include <pangolin/pangolin.h>
#include <pangolin/glglut.h>
#include <pangolin/timer.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Posys/PosysDevice.h>
#include <HAL/Encoder/EncoderDevice.h>
#include <HAL/LIDAR/LIDARDevice.h>

#include <HAL/Utils/GetPot>
#include <HAL/Utils/TicToc.h>

#include <PbMsgs/ImageArray.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Matrix.h>

pangolin::DataLog g_PlotLogAccel;
pangolin::DataLog g_PlotLogGyro;
pangolin::DataLog g_PlotLogMag;

using std::placeholders::_1;

class SensorViewer {
 public:
  SensorViewer() : num_channels_(0), base_width_(0), base_height_(0),
                   has_camera_(false), has_imu_(false), has_posys_(false),
                   has_encoder_(false), has_lidar_(false),
                   is_running_(true), is_stepping_(false), frame_number_(0),
                   panel_height_(0),
                   logger_(pb::Logger::GetInstance())
  {
  }

  virtual ~SensorViewer() {}

  void SetupGUI() {
    panel_height_ = 30;
    int window_width = num_channels_ * base_width_;
    pangolin::OpenGlRenderState render_state;
    pangolin::Handler3D handler(render_state);
    pangolin::CreateWindowAndBind("SensorViewer",
                                  window_width,
                                  base_height_ * (has_imu_ ? 3.0/2.0 : 1.0));

    glPixelStorei(GL_PACK_ALIGNMENT,1);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);

    // Create panel.
    pangolin::CreatePanel("ui").
        SetBounds(0, pangolin::Attach::Pix(panel_height_), 0, 1.0);
    logging_enabled_.reset(new pangolin::Var<bool>("ui.Enable Logging",
                                                   false));

    // Create Smart viewports for each camera image that preserve aspect.
    pangolin::View& cameraView = pangolin::Display("camera");
    cameraView.SetLayout(pangolin::LayoutEqual);
    cameraView.SetBounds(pangolin::Attach::Pix(panel_height_), 1.0, 0, 1.0);
    pangolin::DisplayBase().AddDisplay(cameraView);

    for (size_t ii = 0; ii < num_channels_; ++ii) {
      cameraView.AddDisplay(pangolin::CreateDisplay().
                            SetAspect((double)camera_.Width() / camera_.Height()));
    }

    if (has_imu_) {
      pangolin::View& imuView = pangolin::CreateDisplay().
          SetLayout(pangolin::LayoutEqualVertical);
      g_PlotLogAccel.SetLabels({"Accel X", "Accel Y", "Accel Z"});
      g_PlotLogGyro.SetLabels({"Gyro X", "Gyro Y", "Gyro Z"});
      g_PlotLogMag.SetLabels({"Mag X", "Mag Y", "Mag Z"});

      accel_plot_.reset(new pangolin::Plotter(&g_PlotLogAccel));
      gyro_plot_.reset(new pangolin::Plotter(&g_PlotLogGyro));
      mag_plot_.reset(new pangolin::Plotter(&g_PlotLogMag));
      imuView.AddDisplay(*accel_plot_);
      imuView.AddDisplay(*gyro_plot_);
      imuView.AddDisplay(*mag_plot_);

      accel_plot_->Track();
      gyro_plot_->Track();
      mag_plot_->Track();

      if (has_camera_) {
        cameraView.SetBounds(1.0/3.0, 1.0, 0.0, 1.0);
        imuView.SetBounds(pangolin::Attach::Pix(panel_height_), 1.0/3.0, 0.0, 1.0);
      }
    }

    pangolin::RegisterKeyPressCallback(
        pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT,
        [this]() {
          is_stepping_ = true;
        });

    pangolin::RegisterKeyPressCallback(' ', [&]() {
        is_running_ = !is_running_;
      });
    pangolin::RegisterKeyPressCallback(
        'l', [this]() {
          *logging_enabled_ = !*logging_enabled_;
          frame_number_ = 0;
        });
  }

  void Run() {
    RegisterCallbacks();

    std::vector<pangolin::GlTexture> glTex(num_channels_);
    pangolin::View& cameraView = pangolin::Display("camera");

    pangolin::Timer timer;
    bool got_first_image = false;
    bool capture_success = false;
    std::shared_ptr<pb::ImageArray> last_images;
    for (; !pangolin::ShouldQuit(); ++frame_number_) {
      const bool go = is_running_ || pangolin::Pushed(is_stepping_);

      std::shared_ptr<pb::ImageArray> images = pb::ImageArray::Create();
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

      // Just display the last images if we didn't get new ones.
      if (!go || !capture_success) {
        images = last_images;
      }
      if (got_first_image && !images->Empty()) {
        for (size_t ii = 0; ii < num_channels_; ++ii) {
          std::shared_ptr<pb::Image> img = images->at(ii);
          if (!glTex[ii].tid && num_channels_) {
            GLint internal_format = (img->Format() == GL_LUMINANCE ?
                                     GL_LUMINANCE : GL_RGBA);
            // Only initialise now we know format.
            glTex[ii].Reinitialise(img->Width(), img->Height(),
                                   internal_format, true, 0,
                                   img->Format(), img->Type(), 0);
          }

          cameraView[ii].Activate();
          if (got_first_image && img->data()) {
            glTex[ii].Upload(img->data(), img->Format(), img->Type());
            glTex[ii].RenderToViewportFlipY();
          }
        }
      }

      if (*logging_enabled_ && is_running_) {
        if (pb::Logger::GetInstance().IsLogging() == false) {
#ifdef ANDROID
          logger_.LogToFile("/sdcard/", "sensors");
#else
          logger_.LogToFile("", "sensors");
#endif
        }
        if (capture_success) {
          LogCamera(images.get());
        }
        DrawLoggingIndicator();
      }

      pangolin::FinishFrame();
      last_images = images;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  }

  void set_camera(const std::string& cam_uri) {
    try {
      camera_ = hal::Camera(cam_uri);
    } catch (const hal::DeviceException& e) {
      std::cerr << "SensorViewer: Camera failed to open! Exception: " << e.what() << std::endl;
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

  void set_encoder(const std::string& encoder_uri)
  {
    encoder_ = hal::Encoder(encoder_uri);
    has_encoder_ = true;
  }

  void set_lidar(const std::string& lidar_uri)
  {
    lidar_ = hal::LIDAR(lidar_uri);
    has_lidar_ = true;
  }

 protected:
  void RegisterCallbacks() {
    if (has_posys_) {
      posys_.RegisterPosysDataCallback(std::bind(&SensorViewer::Posys_Handler,
                                                 this, _1));
      std::cout << "- Registering Posys device." << std::endl;
    }

    if (has_imu_) {
      imu_.RegisterIMUDataCallback(
          std::bind(&SensorViewer::IMU_Handler, this, _1));
      std::cout << "- Registering IMU device." << std::endl;
    }

    if (has_encoder_){
      encoder_.RegisterEncoderDataCallback(
            std::bind(&SensorViewer::Encoder_Handler, this, _1));
      std::cout << "- Registering Encoder device." << std::endl;
    }

    if (has_lidar_){
      lidar_.RegisterLIDARDataCallback(
            std::bind(&SensorViewer::LIDAR_Handler, this, _1));
      std::cout << "- Registering LIDAR device." << std::endl;
    }
  }

  /// Draw red circle on bottom left corner for visual cue
  void DrawLoggingIndicator() {
    if (frame_number_ % 60 < 20) {

      pangolin::Display("camera").ActivatePixelOrthographic();
      pangolin::GlState state;
      state.glDisable(GL_DEPTH_TEST);
      state.glDisable(GL_LIGHTING);
      glColor3f(1.0, 0, 0);
      pangolin::glDrawCircle(20, 20, 7);
    }
  }

  void LogCamera(pb::ImageArray* images) {
    if (!has_camera_) return;

    pb::Msg pbMsg;
    pbMsg.set_timestamp(hal::Tic());
    pbMsg.mutable_camera()->Swap(&images->Ref());
    logger_.LogMessage(pbMsg);
  }

  void IMU_Handler(pb::ImuMsg& IMUdata) {
    if (IMUdata.has_accel()) {
      g_PlotLogAccel.Log(IMUdata.accel().data(0),
                         IMUdata.accel().data(1),
                         IMUdata.accel().data(2));
    }
    if (IMUdata.has_gyro()) {
      g_PlotLogGyro.Log(IMUdata.gyro().data(0),
                        IMUdata.gyro().data(1),
                        IMUdata.gyro().data(2));
    }
    if (IMUdata.has_mag()) {
      g_PlotLogMag.Log(IMUdata.mag().data(0),
                       IMUdata.mag().data(1),
                       IMUdata.mag().data(2));
    }

    if (*logging_enabled_) {
      pb::Msg pbMsg;
      pbMsg.set_timestamp(hal::Tic());
      pbMsg.mutable_imu()->Swap(&IMUdata);
      logger_.LogMessage(pbMsg);
    }
  }

  void Posys_Handler(pb::PoseMsg& PoseData) {
    std::cout << "Posys Data: " << PoseData.pose().data(0) << " - " <<
                 PoseData.pose().data(1) << " - " << PoseData.pose().data(2) <<
                 std::endl;
    if (*logging_enabled_) {
      pb::Msg pbMsg;
      pbMsg.set_timestamp(hal::Tic());
      pbMsg.mutable_pose()->Swap(&PoseData);
      logger_.LogMessage(pbMsg);
    }
  }

  void Encoder_Handler(pb::EncoderMsg& EncoderData) {
    std::cout << "Encoder: ";
    for (int ii = 0; ii < EncoderData.label_size(); ++ii) {
      std::cout << EncoderData.label(ii) << ": " << EncoderData.data(ii) <<
                   ", ";
    }
    std::cout << std::endl;

    if (*logging_enabled_){
      pb::Msg pbMsg;
      pbMsg.set_timestamp(hal::Tic());
      pbMsg.mutable_encoder()->Swap(&EncoderData);
      logger_.LogMessage(pbMsg);
    }
  }

  void LIDAR_Handler(pb::LidarMsg& LidarData)
  {
    //std::cout << "Got LIDAR data..." << std::endl;
    if (*logging_enabled_){
      pb::Msg pbMsg;
      pbMsg.set_timestamp(hal::Tic());
      pbMsg.mutable_lidar()->Swap(&LidarData);
      logger_.LogMessage(pbMsg);
    }
  }

 private:
  size_t num_channels_, base_width_, base_height_;
  bool has_camera_, has_imu_, has_posys_, has_encoder_, has_lidar_;
  bool is_running_, is_stepping_;
  int frame_number_;
  int panel_height_;
  hal::Camera camera_;
  hal::IMU imu_;
  hal::Encoder encoder_;
  hal::Posys posys_;
  hal::LIDAR lidar_;
  std::unique_ptr<pangolin::Var<bool> > logging_enabled_;
  std::unique_ptr<pangolin::Plotter> accel_plot_, gyro_plot_, mag_plot_;
  pb::Logger& logger_;
};

int main(int argc, char* argv[]) {
  GetPot cl_args(argc, argv);

  std::string cam_uri = cl_args.follow("", "-cam");
  std::string imu_uri = cl_args.follow("", "-imu");
  std::string posys_uri = cl_args.follow("", "-posys");
  std::string encoder_uri = cl_args.follow("","-encoder");
  std::string lidar_uri = cl_args.follow("","-lidar");

#ifdef ANDROID
  if (cam_uri.empty()) {
    cam_uri = "kitkat://";
  }
  if (imu_uri.empty()) {
    imu_uri = "android://";
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
    viewer.set_posys(posys_uri);
  }

  if (!encoder_uri.empty()) {
    viewer.set_encoder(encoder_uri);
  }

  if (!lidar_uri.empty()) {
    viewer.set_lidar(lidar_uri);
  }

  viewer.SetupGUI();
  viewer.Run();
}
