#include "AndroidDriver.h"

#include <HAL/Devices/DeviceException.h>

#include <thread>
#include <sstream>

#define LOGV(...) ((void)__android_log_print(ANDROID_LOG_VERBOSE,  "HAL", __VA_ARGS__))
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO,  "HAL", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN,  "HAL", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "HAL", __VA_ARGS__))

namespace hal
{

const char camParamsString[] =
    "ae-bracket-hdr=Off;" \
    "ae-bracket-hdr-values=Off,HDR,AE-Bracket;"\
    "antibanding=off;"\
    "antibanding-values=off,50hz,60hz,auto;"\
    "auto-exposure=frame-average;"\
    "auto-exposure-lock=false;"\
    "auto-exposure-lock-supported=false;"\
    "auto-exposure-values=frame-average,center-weighted,spot-metering;"\
    "auto-whitebalance-lock=false;"\
    "auto-whitebalance-lock-supported=false;"\
    "camera-mode=0;"\
    "camera-mode-values=0,1;"\
    "capture-burst-captures-values=2;"\
    "capture-burst-exposures=;"\
    "capture-burst-exposures-values=-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,8,9,10,11,12;"\
    "capture-burst-interval=1;"\
    "capture-burst-interval-max=10;"\
    "capture-burst-interval-min=1;"\
    "capture-burst-interval-supported=true;"\
    "capture-burst-retroactive=0;"\
    "capture-burst-retroactive-max=2;"\
    "contrast=5;"\
    "denoise=denoise-off;"\
    "denoise-values=denoise-off,denoise-on;"\
    "effect=none;"\
    "effect-values=none,mono,negative,solarize,sepia,posterize,whiteboard,blackboard,aqua,emboss,sketch,neon;"\
    "exposure-compensation=0;"\
    "exposure-compensation-step=0.166667;"\
    "face-detection=off;"\
    "face-detection-values=off,on;"\
    "focal-length=4.6;"\
    "focus-areas=(0, 0, 0, 0, 0);"\
    "focus-distances=0.015625,0.25,0.5,Infinity;"\
    "focus-mode=fixed;"\
    "focus-mode-values=fixed,auto,infinity;"\
    "sf-big-af=1;"\
    "hfr-size-values=800x480,640x480;"\
    "histogram=disable;"\
    "histogram-values=enable,disable;"\
    "horizontal-view-angle=54.8;"\
    "iso=auto;"\
    "iso-values=auto,ISO_HJR,ISO100,ISO200,ISO400,ISO800,ISO1600;"\
    "jpeg-quality=85;"\
    "jpeg-thumbnail-height=384;"\
    "jpeg-thumbnail-quality=90;"\
    "jpeg-thumbnail-size-values=512x288,480x288,432x288,512x384,352x288,320x240,176x144,0x0;"\
    "jpeg-thumbnail-width=512;"\
    "lensshade=enable;"\
    "lensshade-values=enable,disable;"\
    "luma-adaptation=3;"\
    "max-contrast=10;"\
    "max-exposure-compensation=12;"\
    "max-num-detected-faces-hw=2;"\
    "max-num-focus-areas=0;"\
    "max-num-metering-areas=0;"\
    "max-saturation=10;"\
    "max-sharpness=30;"\
    "max-zoom=59;"\
    "mce=enable;"\
    "mce-values=enable,disable;"\
    "metering-areas=(0, 0, 0, 0, 0);"\
    "min-exposure-compensation=-12;"\
    "no-display-mode=0;"\
    "num-snaps-per-shutter=1;"\
    "overlay-format=265;"\
    "picture-format=jpeg;"\
    "picture-format-values=jpeg,raw;"\
    "picture-size=640x480;"\
    "picture-size-values=1280x720,800x600,800x480,640x480,352x288,320x240,176x144;"\
    "power-mode=Normal_Power;"\
    "power-mode-supported=true;"\
    "preferred-preview-size-for-video=1280x720;"\
    "preview-format=yuv420sp;"\
    "preview-format-values=yuv420sp,yuv420sp-adreno,yuv420p,yuv420p,nv12;"\
    "preview-fps-range=30000,30000;"\
    "preview-fps-range-values=(30000,30000);"\
    "preview-frame-rate=30;"\
    "preview-frame-rate-mode=frame-rate-auto;"\
    "preview-frame-rate-values=5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120;"\
    "preview-size=1280x720;"\
    "preview-size-values=1280x720,800x480,768x432,720x480,640x480,576x432,480x320,384x288,352x288,320x240,240x160,176x144;"\
    "qc-camera-features=1;"\
    "qc-max-num-requested-faces=2;"\
    "redeye-reduction=disable;"\
    "redeye-reduction-values=enable,disable;"\
    "saturation=5;"\
    "scene-detect=off;"\
    "scene-detect-values=off,on;"\
    "scene-mode=auto;"\
    "scene-mode-values=auto,asd,action,portrait,landscape,night,night-portrait,theatre,beach,snow,sunset,steadyphoto,fireworks,sports,party,candlelight,backlight,flowers,AR;"\
    "selectable-zone-af=auto;"\
    "selectable-zone-af-values=;"\
    "sharpness=10;"\
    "single-isp-output-enabled=false;"\
    "skinToneEnhancement=0;"\
    "skinToneEnhancement-values=enable,disable;"\
    "strtextures=OFF;"\
    "touch-af-aec=touch-off;"\
    "touch-af-aec-values=;"\
    "touchAfAec-dx=100;"\
    "touchAfAec-dy=100;"\
    "vertical-view-angle=42.5;"\
    "video-frame-format=yuv420sp;"\
    "video-hfr=off;"\
    "video-hfr-values=off,60;"\
    "video-size=1280x720;"\
    "video-size-values=1280x720,800x480,720x480,640x480,480x320,352x288,320x240,176x144;"\
    "video-snapshot-supported=true;"\
    "video-zoom-support=true;"\
    "whitebalance=auto;"\
    "whitebalance-values=auto,incandescent,fluorescent,daylight,cloudy-daylight;"\
    "zoom=0;"\
    "zoom-ratios=100,102,104,107,109,112,114,117,120,123,125,128,131,135,138,141,144,148,151,155,158,162,166,170,174,178,182,186,190,195,200,204,209,214,219,224,229,235,240,246,251,257,263,270,276,282,289,296,303,310,317,324,332,340,348,356,364,373,381,390;"\
    "zoom-supported=true;"\
    "zsl=off;"\
    "zsl-values=off,on";


AndroidDriver::AndroidDriver()
{
  // Create texture for preview
  glGenTextures(1, &m_Texture);
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, m_Texture);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER,GL_NEAREST);
  glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER,GL_NEAREST);

  // Check there were no GL errors
  GLenum errCode = glGetError();
  if(errCode != GL_NO_ERROR) {
    LOGE("Error generating texture, GL ERROR CODE: %d\n", errCode);
    throw hal::DeviceException("Error generating texture");
  }

  m_SurfaceTexture = new android::SurfaceTexture( m_Texture );
  m_SurfaceTextureClient = new android::SurfaceTextureClient(m_SurfaceTexture);

  const int numcams = android::Camera::getNumberOfCameras();
  if(numcams < 1) {
    LOGE("No cameras found");
    throw hal::DeviceException("No cameras found");
  }

  int rear_camera = 0;
  struct android::CameraInfo camera_info;
  for (int i = 0; i < numcams; i++) {
    android::Camera::getCameraInfo(i, &camera_info);
    if (camera_info.facing == 0) {
      rear_camera = i;
      break;
    }
  }

  LOGV("Initializing Android camera...\n");
  m_Cam = android::Camera::connect(rear_camera);
  if(m_Cam == NULL || m_Cam->getStatus() != android::NO_ERROR) {
    LOGE("Failed to connect camera");
    throw hal::DeviceException("Failed to connect camera");
  }

  // set up listener.
  m_CamListener = new HALCameraListener();
  m_Cam->setListener( m_CamListener );

  LOGI("%s\n", m_Cam->getParameters().string());

  // set our parameters based on the camParamsString
  android::String8 params8 = android::String8(camParamsString, strlen(camParamsString));
  if( m_Cam->setParameters(params8) != android::NO_ERROR ) {
    LOGE("Failed to set params for camera.\n");
  }

  m_Cam->setPreviewCallbackFlags(CAMERA_FRAME_CALLBACK_FLAG_CAMERA);
//  m_Cam->setPreviewCallbackFlags(CAMERA_FRAME_CALLBACK_FLAG_CAMCORDER);

  android::sp<android::ISurfaceTexture> surftex =
      m_SurfaceTextureClient->getISurfaceTexture();

  if(!surftex.get()) {
    LOGE("Failed to get ISurfaceTexture");
    throw hal::DeviceException("Failed to get ISurfaceTexture");
  }

  if( m_Cam->setPreviewTexture(surftex) != android::NO_ERROR ) {
    LOGE("Error setting gl preview display surface.");
    throw hal::DeviceException("Error setting gl preview display surface.");
  }

  if( m_Cam->startPreview() != android::NO_ERROR ) {
    LOGE("Camera could not start preview.");
    throw hal::DeviceException("Could not start preview.");
  }else{
    LOGV("Android camera started.");
  }

  // Wait until frame data becomes available.
  android::sp<android::GraphicBuffer> gbuf;
  while(!gbuf.get()) {
    m_SurfaceTexture->updateTexImage();
    gbuf = m_SurfaceTexture->getCurrentBuffer();
  }

  // Hack which seems to help camera to start reliably.
  std::this_thread::sleep_for( std::chrono::milliseconds(500) );

  // Get width and height
  m_nWidth  = gbuf->getWidth();
  m_nHeight = gbuf->getHeight();
}

AndroidDriver::~AndroidDriver()
{
  m_Cam->stopPreview();
  m_Cam->disconnect();
}

bool AndroidDriver::Capture( hal::CameraMsg& vImages )
{
  hal::ImageMsg* pbImg = vImages.add_image();

  pbImg->set_width(Width());
  pbImg->set_height(Height());
  pbImg->set_type( hal::PB_UNSIGNED_BYTE );
  pbImg->set_format( hal::PB_LUMINANCE );
  pbImg->set_timestamp( m_CamListener->GetTimestamp() );

  return m_CamListener->GetImages( pbImg );
}

size_t AndroidDriver::NumChannels() const
{
  return 1;
}

size_t AndroidDriver::Width( size_t ) const
{
  return m_nWidth;
}

size_t AndroidDriver::Height( size_t ) const
{
  return m_nHeight;
}

} /* namespace */
