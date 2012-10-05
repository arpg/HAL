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

//    if(m_ImageNode.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT))
//    {
//        //Warp RGB to match depth
//        AlternativeViewPointCapability avpx = m_ImageNode.GetAlternativeViewPointCap();
//        rc = avpx.SetViewPoint(m_DepthNode);
//        if (rc != XN_STATUS_OK) {
//            errors.ToString(strError, 1024);
//            printf("%s\n", strError);
//        }
//    }

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
    if(vImages.size() != 2)
    {
        vImages.resize(2);
        vImages[0].Image = cv::Mat( 480, 640, CV_8UC3 );
        vImages[1].Image = cv::Mat( 480, 640, CV_16UC1 );
    }

    // Get RGB and depthmap data pointers from Kinect
	m_ImageNode.GetMetaData(m_ImageMD);
	m_DepthNode.GetMetaData(m_DepthMD);

    // Set timestamps in camera property map
    m_pPropertyMap->SetProperty( "TimestampRGB", m_ImageMD.Timestamp() );
    m_pPropertyMap->SetProperty( "TimestampDepth", m_DepthMD.Timestamp() );

    //----------------------------------------------------------------------------
    // image 0 is RGB image
    vImages[0].Map.SetProperty( "CameraTime", m_ImageMD.Timestamp() );
    memcpy(vImages[0].Image.data, m_ImageMD.RGB24Data(), m_ImageMD.DataSize() );

    //----------------------------------------------------------------------------
    // image 1 is depth image
    vImages[1].Map.SetProperty( "CameraTime", m_DepthMD.Timestamp() );
    memcpy(vImages[1].Image.data, m_DepthMD.Data(), m_DepthMD.DataSize() );

    return true;
}
