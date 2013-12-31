#include <pangolin/pangolin.h>
#include <pangolin/timer.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Posys/PosysDevice.h>

#include <HAL/Utils/GetPot>
#include <HAL/Utils/TicToc.h>

#include <PbMsgs/Logger.h>
#include <PbMsgs/Matrix.h>

bool        g_bLog      = false;
pb::Logger& g_Logger    = pb::Logger::GetInstance();

pangolin::DataLog g_PlotLogAccel;
pangolin::DataLog g_PlotLogGryo;
pangolin::DataLog g_PlotLogMag;

void IMU_Handler(pb::ImuMsg& IMUdata) {
  if (g_bLog) {
    pb::Msg pbMsg;
    pbMsg.set_timestamp(hal::Tic());
    pbMsg.mutable_imu()->Swap(&IMUdata);
    g_Logger.LogMessage(pbMsg);
  }

  // const pb::VectorMsg& pbVec = IMUdata.accel();
  // printf("X: %5f    Y: %5f     Z: %5f\r",
  // pbVec.data(0),pbVec.data(1),pbVec.data(2));
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
  if (g_bLog) {
    pb::Msg pbMsg;
    pbMsg.set_timestamp(hal::Tic());
    pbMsg.mutable_pose()->Swap(&PoseData);
    g_Logger.LogMessage(pbMsg);
  }
}

int main(int argc, char* argv[]) {
  GetPot clArgs(argc,argv);

  std::string sCam = clArgs.follow("", "-cam");
  const bool bHaveCam = !sCam.empty();
  std::string sIMU = clArgs.follow("", "-imu");
  const bool bHaveIMU = !sIMU.empty();

  ///-------------------- CAMERA INIT (Optional)

  // N cameras, each w*h in dimension, greyscale
  size_t nNumChannels = 0;
  size_t nBaseWidth = 0;
  size_t nBaseHeight = 0;

  hal::Camera theCam;
  pb::ImageArray images;
  if (bHaveCam) {
    theCam = hal::Camera(sCam);
    nNumChannels = theCam.NumChannels();
    nBaseWidth = theCam.Width();
    nBaseHeight = theCam.Height();

    std::cout << "- Opening camera with " << nNumChannels
              << " channel(s)." << std::endl;
    for (size_t ii=0; ii<nNumChannels; ++ii) {
      std::cout << "\t" << theCam.Width(ii) << "x"
                << theCam.Height(ii) << std::endl;
    }
  }

  ///-------------------- IMU INIT (Optional)
  hal::IMU theIMU;
  if (bHaveIMU) {
    theIMU = hal::IMU(sIMU);
    theIMU.RegisterIMUDataCallback(IMU_Handler);
    std::cout << "- Registering IMU device." << std::endl;
  }

  ///-------------------- POSYS INIT (Optional)
  std::string sPosys = clArgs.follow("", "-posys");
  const bool bHavePosys = !sPosys.empty();

  hal::Posys thePosys;
  if (bHavePosys) {
    thePosys = hal::Posys(sPosys);
    thePosys.RegisterPosysDataCallback(Posys_Handler);
    std::cout << "- Registering Posys device." << std::endl;
  }


  ///-------------------- WINDOW INIT

  // Setup OpenGL Display (based on GLUT)
  pangolin::CreateWindowAndBind(__FILE__,
                                nNumChannels * nBaseWidth,
                                nBaseHeight * (bHaveIMU ? 3.0/2.0 : 1.0));
  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  pangolin::GlTexture glTex[nNumChannels];

  // Create Smart viewports for each camera image that preserve aspect
  pangolin::View& cameraView = pangolin::CreateDisplay().SetLayout(pangolin::LayoutEqual);
  for (size_t ii=0; ii < nNumChannels; ++ii) {
    cameraView.AddDisplay(pangolin::CreateDisplay().SetAspect((double)nBaseWidth/nBaseHeight));
  }

  if (bHaveIMU) {
    pangolin::View& imuView = pangolin::CreateDisplay().SetLayout(pangolin::LayoutEqualVertical);
    imuView.AddDisplay(pangolin::CreatePlotter("Accel", &g_PlotLogAccel));
    imuView.AddDisplay(pangolin::CreatePlotter("Gryo", &g_PlotLogGryo));
    imuView.AddDisplay(pangolin::CreatePlotter("Mag", &g_PlotLogMag));

    if (bHaveCam) {
      cameraView.SetBounds(1.0/3.0, 1.0, 0.0, 1.0);
      imuView.SetBounds(0, 1.0/3.0, 0.0, 1.0);
    }
  }

  bool bRun = true;
  bool bStep = false;
  unsigned long nFrame=0;

  pangolin::RegisterKeyPressCallback(
      pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT,
      [&bStep]() {bStep=true;});
  pangolin::RegisterKeyPressCallback(' ', [&]() {bRun = !bRun;});
  pangolin::RegisterKeyPressCallback(
      'l', [&]() { g_bLog = !g_bLog; nFrame = 0; });

  pangolin::Timer theTimer;
  bool got_first_image = false;
  for (; !pangolin::ShouldQuit(); nFrame++) {
    const bool bGo = bRun || pangolin::Pushed(bStep);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    if (bGo && nNumChannels) {
      bool capture_success = theCam.Capture(images);
      if (!got_first_image && capture_success) {
        got_first_image = true;
      }

#ifdef HAVE_GLUT
      if (nFrame % 30 == 0) {
        char buffer[1024];
        sprintf(buffer,"SensorViewer (FPS: %f)", 30.0 / theTimer.Elapsed_s());
        glutSetWindowTitle(buffer);
        theTimer.Reset();
      }
#endif
#if ANDROID
      if (nFrame % 30 == 0) {
        LOGI("SensorViewer (FPS: %f)", 30.0 / theTimer.Elapsed_s());
        theTimer.Reset();
      }
#endif
    }

    if (got_first_image) {
      for (size_t ii = 0; ii < nNumChannels; ++ii) {
        pb::Image img = images[ii];
        if (!glTex[ii].tid && nNumChannels) {
          // Only initialise now we know format.
          glTex[ii].Reinitialise(img.Width(), img.Height(),
                                 GL_RGBA, true, 0,
                                 img.Format(), img.Type(), 0);
        }

        cameraView[ii].Activate();
        if (got_first_image && img.data()) {
          glTex[ii].Upload(img.data(), img.Format(), img.Type());
          glTex[ii].RenderToViewportFlipY();
        }
      }
    }

    if (g_bLog && bRun && bHaveCam) {
      pb::Msg pbMsg;
      pbMsg.set_timestamp(hal::Tic());
      pbMsg.mutable_camera()->Swap(&images.Ref());
      g_Logger.LogMessage(pbMsg);
    }

    if (g_bLog && bRun) {
      // draw red circle on bottom left corner for visual cue
      if (! ((nFrame / 30) %2)) {
        cameraView.ActivatePixelOrthographic();
        pangolin::GlState state;
        state.glDisable(GL_DEPTH_TEST);
        state.glDisable(GL_LIGHTING);
        glColor3f(1.0, 0, 0);
        pangolin::glDrawCircle(20,20,7);
      }
    }

    pangolin::FinishFrame();
  }
}
