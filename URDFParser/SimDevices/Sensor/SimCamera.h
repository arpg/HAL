// Copyright (c) bminortx

#ifndef URDFPARSER_SIMDEVICES_SENSOR_SIMCAMERA_H_
#define URDFPARSER_SIMDEVICES_SENSOR_SIMCAMERA_H_

#include <SimDevices/SimDeviceInfo.h>
#include <SceneGraph/SimCam.h>
#include <calibu/cam/CameraXml.h>
#include <unistd.h>

#include <string>

// A wrapper for SceneGraph::SimCam for LocalSim.

class SimCamera: public SimDeviceInfo {
 public:
  //////////////////////
  // GLOBAL VARIABLES
  //////////////////////

  // Camera Properties
  unsigned int image_width_;
  unsigned int image_height_;
  unsigned int num_channels_;
  int baseline_;
  int fps_;
  SceneGraph::GLSimCam glcamera_;
  calibu::CameraRig glcamera_rig_;
  SceneGraph::eSimCamType glcamera_type_;
  //  Path of model.xml file for the sensor
  std::string model_dir_;


  //////////////////////
  // INITIALIZER
  //////////////////////

  SimCamera(std::string sDeviceName, std::string sBodyName,
            std::string sRobotName, SceneGraph::eSimCamType CameraType, int FPS,
            Eigen::Vector6d vInitPose, std::string sCameraModel) {
    SetDeviceName(sDeviceName);
    SetBodyName(sBodyName);
    SetRobotName(sRobotName);
    m_sDeviceType = "Camera";
    num_channels_ = 1;
    fps_ = FPS;
    glcamera_type_ = CameraType;
    model_dir_ = sCameraModel;
    glcamera_rig_ = calibu::ReadXmlRig(sCameraModel);
    m_vPose = vInitPose;
    m_bDeviceOn = false;
    m_bHasPublished = false;
  }

  bool init(SceneGraph::GLSceneGraph* glGraph) {
    calibu::CameraModel theCam = glcamera_rig_.cameras[0].camera;
    image_width_ = theCam.Width();
    image_height_ = theCam.Height();
    glcamera_.Init(glGraph, _Cart2T(m_vPose), theCam.K(),
                  image_width_, image_height_, glcamera_type_);
    return true;
  }

  //////////////////////
  // CAMERA FUNCTIONS
  //////////////////////

  inline void NormalizeDepth(float* Depth, unsigned int Size) {
    //  find max depth
    float MaxDepth = 0;
    for (unsigned int ii = 0; ii < Size; ii++) {
      if (MaxDepth < Depth[ii]) {
        MaxDepth = Depth[ii];
      }
    }
    if (MaxDepth == 0) {
      return;
    }
    //  Normalize
    for (unsigned int ii = 0; ii < Size; ii++) {
      Depth[ii] = Depth[ii] / MaxDepth;
    }
  }

  ////////
  //  We need two capture functions:
  //  RGB and Grey take char*, while Depth takes float*
  ////////

  bool capture(char* pImgbuf) {
    if (glcamera_type_ == SceneGraph::eSimCamLuminance) {
      return glcamera_.CaptureGrey(pImgbuf);
    } else if (glcamera_type_ == SceneGraph::eSimCamRGB) {
      return glcamera_.CaptureRGB(pImgbuf);
    } else {
      return false;
    }
  }

  bool capture(float* pImgbuf) {
    if (glcamera_type_ == SceneGraph::eSimCamDepth) {
      return glcamera_.CaptureDepth(pImgbuf);
    } else {
      return false;
    }
  }

  //////
  // Update camera images
  //////

  void Update() {
    glcamera_.SetPoseRobot(_Cart2T(m_vPose));
    glcamera_.RenderToTexture();
    glcamera_.DrawCamera();
    // TODO(anyone): Improve this methodology
    usleep(1E0/fps_);
  }

  //  Must set this through the ModelNode set.
  void UpdateByPose(Eigen::Vector6d DesiredPose) {
    m_vPose = DesiredPose;
    glcamera_.SetPoseRobot(_Cart2T(DesiredPose));
    glcamera_.RenderToTexture();
    glcamera_.DrawCamera();
    usleep(1E0/fps_);
  }
};

#endif  // URDFPARSER_SIMDEVICES_SENSOR_SIMCAMERA_H_
