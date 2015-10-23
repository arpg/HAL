#include <iostream>
#include <dc1394/conversions.h>
#include "FlycapDriver.h"

using namespace hal;
using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void FlycapDriver::_CheckError( FlyCapture2::Error err )
{
	if( err != FlyCapture2::PGRERROR_OK ) {
		err.PrintErrorTrace();
		cerr << "Flycapture SDK exception!\n";
	}
}

vector<string> _split(const string &s, char delim ) 
{
	vector<string> elems;
	stringstream ss(s);
	string item;
	while (getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	FlycapDriver::FlycapDriver( PropertyMap& device_properties )
: device_properties_(device_properties)
{
	// TODO(jmf)
	// * Support other formats aside from format_7??
	// * Have adjustable framerates?
	// * Config some GPIO pins.. at least triggerIN and out?
	// * Configure some image properties? gain, etc.
	// * This only work for blackflies... too much hardcoded.

	string sMode        = device_properties_.GetProperty<string>("mode", "FORMAT7_0");
	ImageDim Dims            = device_properties_.GetProperty<ImageDim>("size", ImageDim(1920,1200));
	ImageRoi ROI             = device_properties_.GetProperty<ImageRoi>("roi", ImageRoi(0,0,0,0));
	string sPixelFormat = device_properties_.GetProperty<string>("format", "RGB8");
	string sMethod      = device_properties_.GetProperty<string>("debayer_method", "bilinear");
	string sFilter      = device_properties_.GetProperty<string>("debayer_filter", "rggb");


	FlyCapture2::Mode Mode;
  unsigned int PacketSize = 3024;
	if( sMode == "FORMAT7_1" ) {
		Mode = FlyCapture2::MODE_1;
	} else if( sMode == "FORMAT7_2" ) {
		Mode = FlyCapture2::MODE_2;
	} else if( sMode == "FORMAT7_3" ) {
		Mode = FlyCapture2::MODE_3;
	} else if( sMode == "FORMAT7_4" ) {
		PacketSize = 6480; // for RGB8 
		Dims.x = 960;
		Dims.y = 600;
		Mode = FlyCapture2::MODE_4;
	} else if( sMode == "FORMAT7_5" ) {
		Mode = FlyCapture2::MODE_5;
	} else if( sMode == "FORMAT7_6" ) {
		Mode = FlyCapture2::MODE_6;
	} else if( sMode == "FORMAT7_7" ) {
		Mode = FlyCapture2::MODE_7;
	} else if( sMode == "FORMAT7_8" ) {
		Mode = FlyCapture2::MODE_8;
	} else if( sMode == "FORMAT7_9" ) {
		Mode = FlyCapture2::MODE_9;
	} else if( sMode == "FORMAT7_10" ) {
		Mode = FlyCapture2::MODE_10;
	} else if( sMode == "FORMAT7_11" ) {
		Mode = FlyCapture2::MODE_11;
	} else if( sMode == "FORMAT7_12" ) {
		Mode = FlyCapture2::MODE_12;
	} else if( sMode == "FORMAT7_13" ) {
		Mode = FlyCapture2::MODE_13;
	} else if( sMode == "FORMAT7_14" ) {
		Mode = FlyCapture2::MODE_14;
	} else if( sMode == "FORMAT7_15" ) {
		Mode = FlyCapture2::MODE_15;
	} else {
		Mode = FlyCapture2::MODE_0;
		PacketSize = 8640; // 8640 gives 30fps
    PacketSize = 12096; // 12096 gives 14fps in RGB8 mode at 1920x1200... sigh 
    PacketSize = 24192; // 24192 gives 28fps in RGB8 mode at 1920x1200... sigh 
	  // see packet size set by flycap, and also knowledge base article: ptgrey.com/KB/10350 
	}


	// dc1394bayer_method_t Method;
	if( sMethod == "nearest" ) {
		debayer_method_ = DC1394_BAYER_METHOD_NEAREST;
	} else if( sMethod == "simple" ) {
		debayer_method_ = DC1394_BAYER_METHOD_SIMPLE;
	} else if( sMethod == "bilinear" ) {
		debayer_method_ = DC1394_BAYER_METHOD_BILINEAR;
	} else if( sMethod == "hqlinear" ) {
		debayer_method_ = DC1394_BAYER_METHOD_HQLINEAR;
	} else if( sMethod == "downsample" ) {
		debayer_method_ = DC1394_BAYER_METHOD_DOWNSAMPLE;
	} else if( sMethod == "none" ) {
		debayer_method_ = (dc1394bayer_method_t)0;
	}

	// dc1394color_filter_t Filter;
	if( sFilter == "rggb" ) {
		debayer_filter_ = DC1394_COLOR_FILTER_RGGB;
	} else if( sFilter == "gbrg" ) {
		debayer_filter_ = DC1394_COLOR_FILTER_GBRG;
	} else if( sFilter == "grbg" ) {
		debayer_filter_ = DC1394_COLOR_FILTER_GRBG;
	} else {
		debayer_filter_ = DC1394_COLOR_FILTER_BGGR;
	}

	// pass properties back using driver property map
	if( debayer_method_ != (dc1394bayer_method_t)0 ){
		video_format_ = hal::PB_RGB;
		video_type_   = hal::PB_UNSIGNED_BYTE;
	}

	vector<unsigned int> vID;
	if( device_properties_.GetProperty<string>("ids") != "0" ){
		if( device_properties_.Contains("ids")) {
			string sids = device_properties_.GetProperty("ids");
			if( sids != "0" ){ 
				sids = sids.substr( sids.find("<")+1, sids.find(">")-1 );
				vector<string> ids = _split( sids, ';' );
				for( size_t ii = 0; ii < ids.size(); ii++ ){
					vID.push_back( atoi(ids[ii].c_str()) );
				}
			}
		}
	}


	if( ROI.w == 0 && ROI.h == 0 ) {
		ROI.w = Dims.x;
		ROI.h = Dims.y;
	}


	image_width_  = ROI.w;
	image_height_ = ROI.h;

	FlyCapture2::Error error;

	FlyCapture2::BusManager BusMgr;
	unsigned int            nTotalCams;

	error = BusMgr.GetNumOfCameras(&nTotalCams);
	_CheckError(error);

	if (nTotalCams == 0) {
		cerr << "No cameras found!\n";
		return;
	}

	if (nTotalCams < vID.size()) {
		cerr << "Less cameras detected than those requested!\n";
		return;
	}

	unsigned int nNumCams;
	// If no ids are provided, only one camera will be opened.
	if (vID.empty()) {
		nNumCams = 1;
	} else {
		nNumCams = nTotalCams;
	}

	printf("going to open %d of %d cameras\n", nNumCams, nTotalCams );

/*
	// compute frame rate when in Format7 
	// http://damien.douxchamps.net/ieee1394/libdc1394/faq/v1/#How_can_I_work_out_the_packet_size_for_a_wanted_frame_rate
	float bus_period = 500;
	float frame_rate = 30;
	int num_packets = (int) (1.0/(bus_period*frame_rate) + 0.5);
	int denominator = num_packets*8; 
	int packet_size = (image_width_*image_height_*depth_ + denominator - 1)/denominator;
	printf("computed packet_size %d (bus_period %f; frame_rate %f; num_packets %d; denom %d\n", packet_size, bus_period, frame_rate, num_packets, denominator );
*/

	// prepare Format 7 config
	FlyCapture2::Format7ImageSettings F7Config;

	if( sPixelFormat == "MONO8" ){
		F7Config.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
		depth_ = 8;
	} else if ( sPixelFormat == "MONO16" ){
		F7Config.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO16;
		depth_ = 8;
	} else if ( sPixelFormat == "RAW8" ){
		F7Config.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;
		depth_ = 8;
	} else if ( sPixelFormat == "RAW12" ){
		F7Config.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW12;
		depth_ = 12;
	} else if ( sPixelFormat == "RGB8" ){
		F7Config.pixelFormat = FlyCapture2::PIXEL_FORMAT_RGB8;
		debayer_method_ = (dc1394bayer_method_t)0;
		depth_ = 8;
	} else{
		F7Config.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8;
		depth_ = 8;
	}

	F7Config.mode = Mode;
	F7Config.height = image_height_;
	F7Config.width  = image_width_;
	F7Config.offsetX = ROI.x;
	F7Config.offsetY = ROI.y;


	// image properties
	FlyCapture2::Property P_Shutter;
	P_Shutter.type = FlyCapture2::SHUTTER;
	P_Shutter.onOff = true;
	P_Shutter.autoManualMode = true;
	P_Shutter.absControl = false;
	P_Shutter.absValue = 8;

	FlyCapture2::Property P_Exposure;
	P_Exposure.type = FlyCapture2::AUTO_EXPOSURE;
	P_Exposure.onOff = true;
	P_Exposure.autoManualMode = true;
	P_Exposure.absControl = false;
	P_Exposure.absValue = -3.5;

	FlyCapture2::Property P_Gain;
	P_Gain.type = FlyCapture2::GAIN;
	P_Gain.onOff = true;
	P_Gain.autoManualMode = true;
	P_Gain.absControl = false;
	P_Gain.absValue = 3;

	FlyCapture2::Property P_Brightness;
	P_Brightness.type = FlyCapture2::BRIGHTNESS;
	P_Brightness.onOff = true;
	P_Brightness.autoManualMode = true;
	P_Brightness.absControl = false;
	P_Brightness.absValue = 50;

	FlyCapture2::Property P_Sharpness;
	P_Sharpness.type = FlyCapture2::SHARPNESS;
	P_Sharpness.onOff = false;
	P_Sharpness.autoManualMode = false;
	P_Sharpness.absControl = false;
	P_Sharpness.absValue = 0;

	for(unsigned int ii = 0; ii < nNumCams; ++ii) {
		FlyCapture2::PGRGuid         GUID;

		// look for camera
		if(vID.empty()) {
			error = BusMgr.GetCameraFromIndex(ii, &GUID);
			_CheckError(error);
		} else {
			error = BusMgr.GetCameraFromSerialNumber(vID[ii], &GUID);
			_CheckError(error);
		}
//		cout << "Setting up camera " << ii << endl;

		// connect to camera
		FlyCapture2::Camera* pCam = new FlyCapture2::Camera;
		error = pCam->Connect(&GUID);
		_CheckError(error);

		// see what we can learn about these cameras
		FlyCapture2::CameraInfo CamInfo;
		pCam->GetCameraInfo(&CamInfo);
//		printf("Camera serial number: %d\n", CamInfo.serialNumber );


		// set video mode and framerate
		error = pCam->SetFormat7Configuration(&F7Config, PacketSize);
		_CheckError(error);


		error = pCam->SetProperty( &P_Shutter );
		_CheckError(error);
		error = pCam->SetProperty( &P_Exposure );
		_CheckError(error);
		error = pCam->SetProperty( &P_Gain );
		_CheckError(error);
		error = pCam->SetProperty( &P_Brightness );
		_CheckError(error);
		error = pCam->SetProperty( &P_Sharpness );
		_CheckError(error);

		/*
		// prepare trigger. first camera always strobes, others get trigger.
		if (ii == 0 && false) {
		FlyCapture2::TriggerMode Trigger;

		// external trigger is disabled
		Trigger.onOff = false;
		Trigger.mode = 0;
		Trigger.parameter = 0;
		Trigger.source = 0;

		// set trigger
		error = pCam->SetTriggerMode( &Trigger );
		_CheckError(error);

		// prepare strobe
		FlyCapture2::StrobeControl Strobe;

		// set GPIO pin direction to out
		const unsigned int StrobeOut = 1;
		pCam->SetGPIOPinDirection( StrobeOut, 1 );

		// set GPIO as strobe
		Strobe.onOff = true;
		Strobe.source = StrobeOut;
		Strobe.delay = 0;
		Strobe.duration = 50;
		Strobe.polarity = 0;

		error = pCam->SetStrobe( &Strobe );
		_CheckError(error);
		} else {
		FlyCapture2::TriggerMode Trigger;

		// set GPIO pin direction to in
		const unsigned int TriggerIn = 0;
		pCam->SetGPIOPinDirection(TriggerIn, 0);

		// external trigger is enabled
		Trigger.onOff = true;
		Trigger.mode = 0;
		Trigger.parameter = 0;
		Trigger.source = TriggerIn;

		// set trigger
		error = pCam->SetTriggerMode( &Trigger );
		_CheckError(error);
		}
		 */

		m_vCams.push_back(pCam);
	}

	//	const FlyCapture2::Camera** tmp = m_vCams.const_pointer;
	//	FlyCapture2::Camera::StartSyncCapture(m_vCams.size(), tmp);

	// initiate transmission on all cameras
	for(unsigned int ii = 0; ii < m_vCams.size(); ++ii) {
		error = m_vCams[ii]->StartCapture();
		_CheckError(error);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
FlycapDriver::~FlycapDriver()
{
	for( size_t ii = 0; ii < m_vCams.size(); ii++ ){
		if( m_vCams[ii] ){
			delete m_vCams[ii];
		}
		m_vCams[ii] = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool FlycapDriver::Capture( hal::CameraMsg& vImages )
{
	FlyCapture2::Error error;

	for(unsigned int ii = 0; ii < m_vCams.size(); ++ii) {
		FlyCapture2::Image Image;

		error = m_vCams[ii]->RetrieveBuffer( &Image );

		if( error != FlyCapture2::PGRERROR_OK ) {
			cerr << "HAL: Error grabbing image from camera." << endl;
			cerr << "Error was: " << error.GetDescription() << endl;
			return false;
		} 

		hal::ImageMsg* pbImg = vImages.add_image();

		if( Image.GetCols() != image_width_){
			cerr << "Error: PGR image_width wrong size (" << Image.GetCols() << " != " << image_width_ << ")\n"; 
			return false;
		}
		if( Image.GetRows() != image_height_ ){ 
			cerr << "Error: PGR image_height wrong size (" << Image.GetRows() << " != " << image_height_ << ")\n"; 
			return false;
		}


		if(debayer_method_ == DC1394_BAYER_METHOD_DOWNSAMPLE) {
			pbImg->set_width( image_width_ /2 );
			pbImg->set_height( image_height_ / 2 );
			pbImg->mutable_data()->resize( 3 * Image.GetCols()/2 * Image.GetRows()/2 );
		}
		else{
			pbImg->set_width( image_width_ );
			pbImg->set_height( image_height_ );
			pbImg->mutable_data()->resize( 3 * Image.GetCols() * Image.GetRows() );
		}


		uint8_t* in = (uint8_t*)Image.GetData();
		uint8_t* out = (uint8_t*)pbImg->data().data();
		if( debayer_method_ ){
			if(   depth_ == 8 ) {
				dc1394_bayer_decoding_8bit( in, out, image_width_, image_height_,
						debayer_filter_, debayer_method_ );
			}
			else {
				cerr << "HAL: Error! 16 bit debayering currently not supported." << endl;
			}
		}
		else{
			pbImg->set_data( in, Image.GetDataSize() );
		}

		// set timestamp only from first camera
		if( ii == 0 ) {
			vImages.set_device_time( Image.GetMetadata().embeddedTimeStamp );
		}
	}
	return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::NumChannels() const
{
	return m_vCams.size();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::Width( size_t /*idx*/ ) const
{
	return image_width_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
size_t FlycapDriver::Height( size_t /*idx*/ ) const
{
	return image_height_;
}

