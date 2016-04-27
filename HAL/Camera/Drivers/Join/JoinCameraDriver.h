#pragma once

#include <vector>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>

namespace hal {

class JoinCameraDriver : public CameraDriverInterface
{
public:
  JoinCameraDriver( PropertyMap& device_properties );

  ~JoinCameraDriver();

  bool Capture( hal::CameraMsg& vImages );
  std::shared_ptr<CameraDriverInterface> GetInputDriver() {
    return std::shared_ptr<CameraDriverInterface>();
  }


  void PrintInfo() {}

  hal::Type   PixelType() {
    return video_type_;
  }

  hal::Format PixelFormat() {
    return video_format_;
  }

  size_t NumChannels() const;
  size_t Width( size_t idx = 0 ) const;
  size_t Height( size_t idx = 0 ) const;

private:
  std::vector<std::shared_ptr<CameraDriverInterface>> m_Cameras;
  std::vector<unsigned int> m_nImgWidth;
  std::vector<unsigned int> m_nImgHeight;
  unsigned int              m_nNumChannels;

  hal::Type										video_type_;
  hal::Format									video_format_;

  PropertyMap&						device_properties_;

  class WorkTeam
  {
  public:
    WorkTeam():m_bStopRequested(false){}
    std::vector<bool> m_bWorkerCaptureNotOver;
    void addWorker(std::shared_ptr<CameraDriverInterface>& cam);
    std::vector<hal::CameraMsg>& process();
    /**
     * Stop the work team permanently
     */
    void stopTeam();

  private:
    void waitForWork(size_t workerId);
    void workerDone(size_t workerId);
    void Worker(std::shared_ptr<CameraDriverInterface>& cam,
                size_t workerId);
    std::vector<std::thread> m_Workers;
    std::vector<bool> m_bWorkerDone;
    std::vector<hal::CameraMsg> m_ImageData;
    std::mutex m_Mutex;
    std::condition_variable m_WorkerCond, m_MasterCond;
    bool m_bStopRequested;
  };

  WorkTeam                  m_WorkTeam;

};

}
