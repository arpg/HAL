#pragma once

#include <vector>
#include <memory>
#include <string>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
/*This depth_registration is really a hack at this point. Inserting the code here that is
  missing from latest libfreenect2 driver. The Dorian3 version contained an OpenCV-compatible
  version of depth-to-rgb registration. The new libreenect2 driver has one that
  is not opencv compatible, but seems to be initialized in a more proper way (using
  rgb & ir cam params), whereas the code in Dorian3 is initialized with hard-coded values
  for camera parameters.*/
#include "depth_registration.h"

#include "HAL/Camera/CameraDriverInterface.h"



namespace hal {

struct RegisteredFreenectCam {
	RegisteredFreenectCam(
			std::shared_ptr<libfreenect2::Freenect2Device> dev,
			std::shared_ptr<libfreenect2::SyncMultiFrameListener> lis,
			uint64_t ser) :
					device(dev),
					listener(lis),
					serialNumber(ser) {
	}
	std::shared_ptr<libfreenect2::Freenect2Device> device;
	std::unique_ptr<DepthRegistration> registration;
	std::shared_ptr<libfreenect2::SyncMultiFrameListener> listener;
	uint64_t serialNumber;
};

class Freenect2Driver: public CameraDriverInterface {
public:
	Freenect2Driver(unsigned int nWidth, unsigned int nHeight, bool bCaptureRGB,
			bool bCaptureDepth, bool bCaptureIR, bool bColor, bool bAlign, std::string sPipeline);

	virtual ~Freenect2Driver();

	bool Capture(hal::CameraMsg& vImages);
	std::shared_ptr<CameraDriverInterface> GetInputDevice() {
		return std::shared_ptr<CameraDriverInterface>();
	}

	std::string GetDeviceProperty(const std::string& sProperty);

	size_t NumChannels() const;
	size_t Width(size_t idx = 0) const;
	size_t Height(size_t idx = 0) const;

private:
	static uint64_t ParseSerialNumber(const std::string& serial);

private:
	static const unsigned int IR_IMAGE_WIDTH;
	static const unsigned int IR_IMAGE_HEIGHT;
	unsigned int m_nImgWidth;
	unsigned int m_nImgHeight;
	bool m_bRGB, m_bDepth, m_bIR, m_bColor, m_bAlign;
	std::vector<std::pair<int,int>> m_dimensions;
	libfreenect2::Freenect2 m_freenect2;
	std::vector<RegisteredFreenectCam> m_devices;
};

}
