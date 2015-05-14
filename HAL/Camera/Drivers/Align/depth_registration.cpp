/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "depth_registration.h"
#include "depth_registration_cpu.h"
#if WITH_OPENCL_SUPPORT
#include "depth_registration_opencl.h"
#endif

DepthRegistration::DepthRegistration()
{
  m_bInit = false;
}

DepthRegistration::~DepthRegistration()
{
}

bool DepthRegistration::init(const std::shared_ptr<calibu::CameraInterface<double>> camColor,
                             const std::shared_ptr<calibu::CameraInterface<double>> camDepth,
                             const Sophus::SE3Group<double> T_cd,
                             const cv::Mat &rotation,
                             const cv::Mat &translation,
                             const cv::Mat &mapX, const cv::Mat &mapY)
{
  SetCameraInfo(camColor, camDepth, T_cd);
  this->rotation = rotation;
  this->translation = translation;
  this->mapX = mapX;
  this->mapY = mapY;
  m_bInit = true;
  return init();
}


void DepthRegistration::SetCameraInfo(const std::shared_ptr<calibu::CameraInterface<double>> camColor,
                                      const std::shared_ptr<calibu::CameraInterface<double>> camDepth,
                                      const Sophus::SE3Group<double> T_cd)
{
    Eigen::Matrix<double,3,3> K;

  K = camColor->K();
  cameraMatrixColor = (cv::Mat_<double>(3,3) <<
                       K(0,0), K(0,1), K(0,2),
                       K(1,0), K(1,1), K(1,2),
                       K(2,0), K(2,1), K(2,2));

  K = camDepth->K();
  cameraMatrixDepth = (cv::Mat_<double>(3,3) <<
                       K(0,0), K(0,1), K(0,2),
                       K(1,0), K(1,1), K(1,2),
                       K(2,0), K(2,1), K(2,2));

  translation = (cv::Mat_<double>(3,1) <<
                 T_cd.translation()(0,0), T_cd.translation()(1,0),
                 T_cd.translation()(2,0));

  auto R = T_cd.rotationMatrix();
  rotation = (cv::Mat_<double>(3,3) <<
              R(0,0), R(0,1), R(0,2),
              R(1,0), R(1,1), R(1,2),
              R(2,0), R(2,1), R(2,2));
}


DepthRegistration *DepthRegistration::New(const cv::Size &color,
                                          const cv::Size &depth,
                                          const cv::Size &raw,
                                          const float zNear, const float zFar,
                                          const float zDist, Method method)
{
  if(method == DEFAULT)
  {
#if WITH_OPENCL_SUPPORT
    method = OPENCL;
#else
    method = CPU;
#endif
  }

  switch(method)
  {
  case DEFAULT:
    throw std::string("No default depth registration method available");
  case CPU:
    return new DepthRegistrationCPU(color, depth, raw, zNear, zFar);
  case OPENCL:
#if WITH_OPENCL_SUPPORT
    return new DepthRegistrationOpenCL(color, depth, raw, zNear, zFar, zDist);
#else
    throw std::string("OpenCL registration method not available");
#endif
  }
  return NULL;
}
