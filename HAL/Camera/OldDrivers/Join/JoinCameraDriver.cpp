#include <iostream>
#include <memory>
#include <string>
#include <algorithm>
#include <thread>

#include <HAL/Devices/DriverFactory.h>
#include <HAL/Utils/TicToc.h>

#include "JoinCameraDriver.h"

using namespace hal;

std::vector<Uri> _SplitUri(const std::string& url)
{
  const char C = '&'; // split token
  std::vector<Uri> ret;
  std::string::size_type begin = 0, end = 0;
  for(; end != std::string::npos; begin = end + 1 ) {
    end = url.find( C, begin );
    std::string s;
    if( end == std::string::npos )
      s = url.substr(begin);
    else
      s = url.substr(begin, end - begin);
    if( !s.empty() ) ret.emplace_back( Uri(s) );
  }
  return ret;
}


JoinCameraDriver::JoinCameraDriver(
    const Uri& uri
    )
{
  
  std::vector<Uri> suburis = _SplitUri(uri.url);
  std::vector<std::shared_ptr<CameraDriverInterface> > cameras;
  m_Cameras.reserve(suburis.size());

  for( const Uri& splituri : suburis ) {
//    std::cout << "Creating stream from uri: " << splituri.ToString()
//      << std::endl;
    m_Cameras.emplace_back
      (DeviceDriverRegistry<hal::CameraDriverInterface>::Instance().Create(splituri));
  }

  if( m_Cameras.empty() ) {
    std::cerr << "JoinCamerDrive: no input cameras given to join\n";
    return;
  }

  /*
  const size_t N = NumChannels();
  m_nImgWidth.reserve(N);
  m_nImgHeight.reserve(N);
  m_nNumChannels = 0;
  */

  for( auto& cam : m_Cameras ) {
    for( size_t i = 0; i < cam->NumChannels(); ++i ) {
      m_nImgWidth.push_back(cam->Width(i));
      m_nImgHeight.push_back(cam->Height(i));
    }
    m_nNumChannels += cam->NumChannels();
    m_WorkTeam.addWorker(cam);
  }
}

JoinCameraDriver::~JoinCameraDriver()
{
	m_WorkTeam.stopTeam();
}

void JoinCameraDriver::WorkTeam::addWorker(
    std::shared_ptr<CameraDriverInterface>& cam)
{
  const size_t workerId = m_ImageData.size();
  {
    std::unique_lock<std::mutex>(m_Mutex);
    m_ImageData.push_back(hal::CameraMsg());
    m_bWorkerDone.push_back(true);
    m_bWorkerCaptureNotOver.push_back(true);
  }
  m_Workers.emplace_back(std::thread(&JoinCameraDriver::WorkTeam::Worker, this,
        std::ref(cam), workerId));
}

  void
JoinCameraDriver::WorkTeam::Worker(std::shared_ptr<CameraDriverInterface>& cam,
    size_t workerId)
{
  for (;!m_bStopRequested;) {
    waitForWork(workerId);
    m_bWorkerCaptureNotOver[workerId] = cam->Capture(m_ImageData[workerId]);
    workerDone(workerId);
  }
  cam.reset();
}

void JoinCameraDriver::WorkTeam::waitForWork(size_t workerId)
{
  std::unique_lock<std::mutex> lock(m_Mutex);
  if( m_bWorkerDone[workerId] ) m_WorkerCond.wait(lock);
}

void JoinCameraDriver::WorkTeam::workerDone(size_t workerId)
{
  std::lock_guard<std::mutex> lock(m_Mutex);
  m_bWorkerDone[workerId] = true;
  m_MasterCond.notify_one();
}

void JoinCameraDriver::WorkTeam::stopTeam(){
	m_bStopRequested = true;
	//let workers loop one iteration
	process();
	for(size_t i = 0; i < m_Workers.size(); i++){
		m_Workers[i].join();
	}
}

bool JoinCameraDriver::Capture( hal::CameraMsg& vImages )
{
  vImages.Clear();
  const double time = Tic();
  vImages.set_system_time(time);
  vImages.set_device_time(time);
  unsigned activeWorkerCount = 0;

  std::vector<hal::CameraMsg>& results = m_WorkTeam.process();

  int ixResult = 0;
  for( hal::CameraMsg& result : results ) {
	if(m_WorkTeam.m_bWorkerCaptureNotOver[ixResult]){
      for( int i = 0; i < result.image_size(); ++i ) {
    	  vImages.add_image()->Swap(result.mutable_image(i));
    	  activeWorkerCount++;
      }
      result.Clear();
    }
	ixResult++;
  }
  return activeWorkerCount == results.size();
}

std::vector<hal::CameraMsg>& JoinCameraDriver::WorkTeam::process()
{
  std::unique_lock<std::mutex> lock(m_Mutex);
  std::fill(m_bWorkerDone.begin(), m_bWorkerDone.end(), false);
  m_WorkerCond.notify_all();
  m_MasterCond.wait(lock, [this]{
      return std::find(m_bWorkerDone.begin(), m_bWorkerDone.end(), false)
      == m_bWorkerDone.end();
      });
  return m_ImageData;
}

std::string JoinCameraDriver::GetProperty(const std::string&)
{
  return std::string();
}

size_t JoinCameraDriver::NumChannels() const
{
  return m_nNumChannels;
}

size_t JoinCameraDriver::Width( size_t idx ) const
{
  return m_nImgWidth[idx];
}

size_t JoinCameraDriver::Height( size_t idx ) const
{
  return m_nImgHeight[idx];
}
