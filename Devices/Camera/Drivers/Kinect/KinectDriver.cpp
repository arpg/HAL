/*
   \file KinectDriver.cpp
 */

#include "KinectDriver.h"

#include <opencv/cv.h>	// for Mat structure

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

	// focal length in mm
	XnUInt64 depth_focal_length_SXGA_mm;   //in mm
	m_DepthNode.GetIntProperty ("ZPD", depth_focal_length_SXGA_mm);

	// pixel size in mm
	double pixelSize;
	m_DepthNode.GetRealProperty ( "ZPPS", pixelSize );  // in mm

	// focal length in pixels
	float depth_focal_length_SXGA = static_cast<float>(depth_focal_length_SXGA_mm / pixelSize);
	m_pPropertyMap->SetProperty( "DepthFocalLength", depth_focal_length_SXGA / 2 );

	/*
	max_depth = m_DepthNode.GetDeviceMaxDepth ();

	XnDouble baseline_local;
	depth.GetRealProperty ("LDDIS", baseline_local);

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
bool KinectDriver::Capture( std::vector<cv::Mat>& vImages )
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

    //----------------------------------------------------------------------------
    // image 0 is RGB image
    vImages[0] = cv::Mat( 480, 640, CV_8UC3 );

	// get RGB image data
	m_ImageNode.GetMetaData(m_ImageMD);

    const XnRGB24Pixel* pImageRow = m_ImageMD.RGB24Data();

    for (unsigned int y = 0; y < m_ImageMD.YRes(); ++y)
    {
        const XnRGB24Pixel* pImage = pImageRow;

        for (unsigned int x = 0; x < m_ImageMD.XRes(); ++x, ++pImage)
        {
            unsigned int idx = x * 3;
            vImages[0].at<unsigned char>(y,idx) = pImage->nBlue;
            vImages[0].at<unsigned char>(y,idx+1) = pImage->nGreen;
            vImages[0].at<unsigned char>(y,idx+2) = pImage->nRed;
        }

        pImageRow += m_ImageMD.XRes();
    }

    //----------------------------------------------------------------------------
    // image 1 is depth image
    vImages[1] = cv::Mat( 480, 640, CV_16UC1 );

	// get depth image data
	m_DepthNode.GetMetaData(m_DepthMD);

    const XnDepthPixel* pDepthRow = m_DepthMD.Data();

    for (unsigned int y = 0; y < m_DepthMD.YRes(); ++y)
    {
        const XnDepthPixel* pDepth = pDepthRow;
        for (unsigned int x = 0; x < m_DepthMD.XRes(); ++x, ++pDepth)
        {
			vImages[1].at<unsigned short>(y,x) = *pDepth;
        }
        pDepthRow += m_DepthMD.XRes();
    }

    return true;
}