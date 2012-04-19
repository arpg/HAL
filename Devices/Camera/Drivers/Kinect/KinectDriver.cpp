/*
   \file KinectDriver.cpp
 */

#include "KinectDriver.h"

#define MAX_DEPTH 10000

using namespace xn;

///////////////////////////////////////////////////////////////////////////////
//
KinectDriver::KinectDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
//
KinectDriver::~KinectDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
//
bool KinectDriver::Init()
{
	XnMapOutputMode MapMode;
	MapMode.nXRes = XN_VGA_X_RES;
	MapMode.nYRes = XN_VGA_Y_RES;
	MapMode.nFPS = 30;

	XnStatus rc;
	EnumerationErrors errors;
	XnChar strError[1024];

	rc = m_Context.Init ();

	if (rc != XN_STATUS_OK) {
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}

	// we need this part if we have multiple kinects connected
	// we would pass an ID (int) to initialize a particular one
	/*
	NodeInfoList NodeList;
	rc = m_Context.EnumerateProductionTrees ( XN_NODE_TYPE_DEVICE, NULL, NodeList, 0 );
	if (rc != XN_STATUS_OK) {
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}

	NodeInfoList::Iterator it = devicesList.Begin ();
	for (int i = 0; i < device; ++i)
		it++;

	NodeInfo NodeInf = *it;

	ProductionNode ProdNode;

	rc = m_Context.CreateProductionTree ( NodeInf, ProdNode );
	if (rc != XN_STATUS_OK) {
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}
	*/

	// create depth generator
	rc = m_DepthNode.Create(m_Context);
	if (rc != XN_STATUS_OK) {
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}
	rc = m_DepthNode.SetMapOutputMode(MapMode);

	// create image generator
	rc = m_ImageNode.Create(m_Context);
	if (rc != XN_STATUS_OK) {
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
	} else {
		rc = m_ImageNode.SetMapOutputMode(MapMode);
	}

	// --- GET CAMERA PARAMS ---
    // Details can be found in XnStreamParams.h

	// focal length in mm
	XnUInt64 depth_focal_length_SXGA_mm;   //in mm
	m_DepthNode.GetIntProperty ("ZPD", depth_focal_length_SXGA_mm);

	// pixel size in mm
	double pixelSize;
	m_DepthNode.GetRealProperty ( "ZPPS", pixelSize );  // in mm

	// focal length in pixels
	float depth_focal_length_SXGA = static_cast<float>(depth_focal_length_SXGA_mm / pixelSize);
	m_pPropertyMap->SetProperty( "DepthFocalLength", depth_focal_length_SXGA / 2 );

	XnDouble dBaselineRGBDepth;
	m_DepthNode.GetRealProperty( "DCRCDIS", dBaselineRGBDepth );
	m_pPropertyMap->SetProperty( "RGBDepthBaseline", dBaselineRGBDepth );

	/*
	max_depth = m_DepthNode.GetDeviceMaxDepth ();


	XnUInt64 shadow_value_local;
	depth.GetIntProperty ("ShadowValue", shadow_value_local);
	shadow_value = (int)shadow_value_local;

	XnUInt64 no_sample_value_local;
	depth.GetIntProperty ("NoSampleValue", no_sample_value_local);
	no_sample_value = (int)no_sample_value_local;

	// baseline from cm -> mm
	baseline = (float)(baseline_local * 10);
	*/

	// --- END GETTING PARAMS ---

	rc = m_Context.StartGeneratingAll();
	if (rc != XN_STATUS_OK) {
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
//
bool KinectDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{
	XnStatus rc = XN_STATUS_OK;

	// Read a new frame
	rc = m_Context.WaitAnyUpdateAll();
	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		return false;
	}

	// prepare return images
    vImages.clear();
    vImages.resize(2);

    // Get data pointers from Kinect
	// get RGB image data
	m_ImageNode.GetMetaData(m_ImageMD);
	// get depth image data
	m_DepthNode.GetMetaData(m_DepthMD);

    const XnRGB24Pixel* pImageRow = m_ImageMD.RGB24Data();
    const XnDepthPixel* pDepthRow = m_DepthMD.Data();

    long unsigned int TimeStampRGB = m_ImageMD.Timestamp();
	m_pPropertyMap->SetProperty( "TimestampRGB", TimeStampRGB );

    long unsigned int TimeStampDepth = m_DepthMD.Timestamp();
	m_pPropertyMap->SetProperty( "TimestampDepth", TimeStampDepth );

    //----------------------------------------------------------------------------
    // image 0 is RGB image
    vImages[0].Image = cv::Mat( 480, 640, CV_8UC3 );
    vImages[0].Map.SetProperty( "CameraTime", TimeStampRGB );

    for (unsigned int y = 0; y < m_ImageMD.YRes(); ++y)
    {
        const XnRGB24Pixel* pImage = pImageRow;

        for (unsigned int x = 0; x < m_ImageMD.XRes(); ++x, ++pImage)
        {
            unsigned int idx = x * 3;
            vImages[0].Image.at<unsigned char>(y,idx) = pImage->nBlue;
            vImages[0].Image.at<unsigned char>(y,idx+1) = pImage->nGreen;
            vImages[0].Image.at<unsigned char>(y,idx+2) = pImage->nRed;
        }

        pImageRow += m_ImageMD.XRes();
    }

    //----------------------------------------------------------------------------
    // image 1 is depth image
    vImages[1].Image = cv::Mat( 480, 640, CV_16UC1 );
    vImages[1].Map.SetProperty( "CameraTime", TimeStampDepth );

    for (unsigned int y = 0; y < m_DepthMD.YRes(); ++y)
    {
        const XnDepthPixel* pDepth = pDepthRow;
        for (unsigned int x = 0; x < m_DepthMD.XRes(); ++x, ++pDepth)
        {
			vImages[1].Image.at<unsigned short>(y,x) = *pDepth;
        }
        pDepthRow += m_DepthMD.XRes();
    }

    return true;
}
