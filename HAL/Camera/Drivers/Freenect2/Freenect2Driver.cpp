#include <iostream>
#include <memory>
#include <string>

#include <libfreenect2/libfreenect2.hpp>
//#include <libfreenect2/threading.h>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <opencv2/opencv.hpp>

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include "Freenect2Driver.h"

using namespace hal;

// values given by libfreenect
const unsigned int Freenect2Driver::IR_IMAGE_WIDTH = 512;
const unsigned int Freenect2Driver::IR_IMAGE_HEIGHT = 424;

Freenect2Driver::Freenect2Driver(unsigned int nWidth, unsigned int nHeight,
		bool bCaptureRGB, bool bCaptureDepth, bool bCaptureIR, bool bColor,
				 bool bAlign, std::string sPipeline) :
				m_nImgWidth(nWidth),
				m_nImgHeight(nHeight),
				m_bRGB(bCaptureRGB),
				m_bDepth(bCaptureDepth),
				m_bIR(bCaptureIR),
				m_bColor(bColor),
				m_bAlign(bAlign) {
	const int N = m_freenect2.enumerateDevices();
	if (N == 0) {
		throw DeviceException("Freenect2 could not find any camera");
	}

	unsigned int types = 0;
	if (bCaptureRGB)
		types |= libfreenect2::Frame::Color;
	if (bCaptureIR)
		types |= libfreenect2::Frame::Ir;
	if (bCaptureDepth)
		types |= libfreenect2::Frame::Depth;
	if (types == 0) {
		throw DeviceException("Freenect2: no channel given (rgb, ir, depth)");
	}

	m_devices.reserve(N);
	//m_listeners.reserve(N);
	//m_lSerialNumbers.reserve(N);
	libfreenect2::PacketPipeline *pipeline = 0;
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
	if (sPipeline.compare("opencl") == 0)
	  {
	    pipeline = new libfreenect2::OpenCLPacketPipeline();
	  }
	else
#endif
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
	  if (sPipeline.compare("opengl") == 0)
	    {
	      pipeline = new libfreenect2::OpenGLPacketPipeline();
	    }
	else
#endif
	  if (sPipeline.compare("cpu") == 0)
	   pipeline = new libfreenect2::CpuPacketPipeline();
	else
	  {
	    throw DeviceException("Unknown packet pipeline");
	  }
	
	for (int i = 0; i < N; ++i) {
		std::shared_ptr<libfreenect2::Freenect2Device> d(
								 m_freenect2.openDevice(i,pipeline));
		if (!d)
			throw DeviceException(
					"Freenect2 could not open device "
							+ m_freenect2.getDeviceSerialNumber(i));

		std::shared_ptr<libfreenect2::SyncMultiFrameListener> listener =
				std::make_shared<libfreenect2::SyncMultiFrameListener>(types);

		if (bCaptureRGB) {
			d->setColorFrameListener(listener.get());
			m_dimensions.push_back(std::make_pair(m_nImgWidth, m_nImgHeight));
		}

		if (bCaptureIR || bCaptureDepth) {
			d->setIrAndDepthFrameListener(listener.get());
			if (bCaptureIR)
				m_dimensions.push_back(
						std::make_pair(IR_IMAGE_WIDTH, IR_IMAGE_HEIGHT));
			if (bCaptureDepth)
				m_dimensions.push_back(
						bAlign ?
								std::make_pair(m_nImgWidth, m_nImgHeight) :
								std::make_pair(IR_IMAGE_WIDTH,
										IR_IMAGE_HEIGHT));
		}
		m_devices.emplace_back(d, listener,
						ParseSerialNumber(d->getSerialNumber()));
		if(bAlign)
			 {
			m_devices[m_devices.size()-1].registration.reset(DepthRegistration::New
		                     (cv::Size(m_nImgWidth, m_nImgHeight),
		                      cv::Size(IR_IMAGE_WIDTH, IR_IMAGE_HEIGHT),
		                      cv::Size(IR_IMAGE_WIDTH, IR_IMAGE_HEIGHT),
		                      0.5f, 20.0f, 0.015f, DepthRegistration::CPU));

		    cv::Mat cameraMatrixColor, cameraMatrixDepth;
		    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
		    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
		    m_devices[m_devices.size()-1].registration->ReadDefaultCameraInfo(cameraMatrixColor, cameraMatrixDepth);

		    m_devices[m_devices.size()-1].registration->init(cameraMatrixColor, cameraMatrixDepth,
		                   cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 3, CV_64F),
		                   cv::Mat::zeros(IR_IMAGE_HEIGHT, IR_IMAGE_WIDTH, CV_32F),
		                   cv::Mat::zeros(IR_IMAGE_HEIGHT, IR_IMAGE_WIDTH, CV_32F));
			 }

		d->start();
	}

}

Freenect2Driver::~Freenect2Driver() {
	for (auto& pdev : m_devices) {
		pdev.device->stop();
		pdev.device->close();
	}
}

uint64_t Freenect2Driver::ParseSerialNumber(const std::string& serial) {
	try {
		return std::strtoull(serial.c_str(), nullptr, 10);
	} catch (...) {
		return 0;
	}
}

bool Freenect2Driver::Capture(hal::CameraMsg& vImages) {
	vImages.Clear();
	vImages.set_device_time(Tic());

	libfreenect2::FrameMap frames;
	libfreenect2::FrameMap::const_iterator fit;

	for (size_t i = 0; i < m_devices.size(); ++i) {
		m_devices[i].listener->waitForNewFrame(frames);
		const double time = Tic();
		bool save_rgb = false;

		if ((fit = frames.find(libfreenect2::Frame::Color)) != frames.end()) {
			hal::ImageMsg* pbImg = vImages.add_image();
			pbImg->set_timestamp(time);
			pbImg->set_width(m_nImgWidth);
			pbImg->set_height(m_nImgHeight);
			pbImg->set_serial_number(m_devices[i].serialNumber);

			
			pbImg->set_type(hal::PB_UNSIGNED_BYTE);
			if (m_bColor)
				pbImg->set_format(hal::PB_BGR);
			else
				pbImg->set_format(hal::PB_LUMINANCE);

			const libfreenect2::Frame* frame = fit->second;

			cv::Mat rgbaTrg(frame->height, frame->width, CV_8UC4, frame->data);
			//From http://fixall.online/well--basically-cvtcolor-only-works-with-a-fixed-set/2220695/
			
			std::vector<cv::Mat1b> channels_rgba;
			cv::split(rgbaTrg, channels_rgba);
			// create matrix with first three channels only
			std::vector<cv::Mat1b> channels_rgb(channels_rgba.begin(),
							    channels_rgba.begin()+3);
			cv::Mat3b trg;
			cv::merge(channels_rgb, trg);

			if (frame->height != m_nImgHeight || frame->width != m_nImgWidth)
				cv::resize(trg, trg, cv::Size(m_nImgWidth, m_nImgHeight));
			cv::flip(trg, trg, 1);
			cv::Mat1b gray;
			if (!m_bColor)
			  {
			    cv::cvtColor(trg,gray, CV_BGR2GRAY,1);
			    //Only copy one gray plane
			    pbImg->set_data(gray.ptr<unsigned char>(),
					gray.rows * gray.cols * 1);
			  }
			else
			  {
			    pbImg->set_data(trg.ptr<unsigned char>(),
					    trg.rows * trg.cols * trg.channels());
			  }
			save_rgb = true;
		}

		if ((fit = frames.find(libfreenect2::Frame::Ir)) != frames.end()) {
			const libfreenect2::Frame* frame = fit->second;
			hal::ImageMsg* pbImg = vImages.add_image();
			pbImg->set_timestamp(time);
			pbImg->set_width(frame->width);
			pbImg->set_height(frame->height);
			pbImg->set_serial_number(m_devices[i].serialNumber);

			pbImg->set_type(hal::PB_FLOAT);
			pbImg->set_format(hal::PB_LUMINANCE);

			cv::Mat trg(frame->height, frame->width, CV_32F, frame->data);
			cv::flip(trg, trg, 1);
			pbImg->set_data(trg.ptr<float>(),
					trg.rows * trg.cols * sizeof(float));
		}

		if ((fit = frames.find(libfreenect2::Frame::Depth)) != frames.end()) {
			const libfreenect2::Frame* frame = fit->second;
			cv::Mat trg = cv::Mat(frame->height, frame->width, CV_32FC1,
					frame->data);

			if (save_rgb && m_bAlign) {
				if (m_nImgHeight != IR_IMAGE_HEIGHT
						|| m_nImgWidth != IR_IMAGE_WIDTH)
					m_devices[i].registration->depthToRGBResolution(trg, trg);
			}

			// change rgb and depth image
			hal::ImageMsg* pbImg = vImages.add_image();
			pbImg->set_timestamp(time);
			pbImg->set_width(trg.cols);
			pbImg->set_height(trg.rows);
			pbImg->set_serial_number(m_devices[i].serialNumber);

			pbImg->set_type(hal::PB_FLOAT);
			pbImg->set_format(hal::PB_LUMINANCE);

			cv::flip(trg, trg, 1);
			pbImg->set_data(trg.ptr<float>(),
					trg.rows * trg.cols * sizeof(float));
		}

		m_devices[i].listener->release(frames);
	}

	return true;
}

std::string Freenect2Driver::GetDeviceProperty(const std::string&) {
	return std::string();
}

size_t Freenect2Driver::NumChannels() const {
	int channels_per_device = 0;
	if (m_bRGB)
		++channels_per_device;
	if (m_bIR)
		++channels_per_device;
	if (m_bDepth)
		++channels_per_device;
	return m_devices.size() * channels_per_device;
}

size_t Freenect2Driver::Width(size_t idx) const {
	return m_dimensions[idx].first;
}

size_t Freenect2Driver::Height(size_t idx) const {
	return m_dimensions[idx].second;
}
