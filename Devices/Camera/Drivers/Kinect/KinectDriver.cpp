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
    m_bGetRGB   = m_pPropertyMap->GetProperty( "GetRGB", true );
    m_bGetDepth = m_pPropertyMap->GetProperty( "GetDepth", true );

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

    m_nNumImgs = 0;

    // create depth generator
    if( m_bGetDepth ) {
        rc = m_DepthNode.Create(m_Context);
        if (rc != XN_STATUS_OK) {
            errors.ToString(strError, 1024);
            printf("%s\n", strError);
            return false;
        }
        rc = m_DepthNode.SetMapOutputMode(MapMode);

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

        m_nNumImgs++;
    }

    // create image generator
    if( m_bGetRGB ) {
        rc = m_ImageNode.Create(m_Context);
        if (rc != XN_STATUS_OK) {
            errors.ToString(strError, 1024);
            printf("%s\n", strError);
        } else {
            rc = m_ImageNode.SetMapOutputMode(MapMode);
            m_nNumImgs++;
        }
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
    rc = m_Context.WaitAndUpdateAll();
    if (rc != XN_STATUS_OK)
    {
        printf("Read failed: %s\n", xnGetStatusString(rc));
        return false;
    }

    // prepare return images
    if( vImages.size() != m_nNumImgs )
    {
        vImages.resize( m_nNumImgs );

        if( m_bGetRGB ) {
            vImages[0].Image = cv::Mat( 480, 640, CV_8UC3 );
        }

        if( m_bGetDepth ) {
            if( m_bGetRGB == false ) {
                vImages[0].Image = cv::Mat( 480, 640, CV_16UC1 );
            } else {
                vImages[1].Image = cv::Mat( 480, 640, CV_16UC1 );
            }
        }
    }

    // Get RGB and depthmap data pointers from Kinect
    if( m_bGetRGB ) {
        m_ImageNode.GetMetaData(m_ImageMD);

        // image 0 is RGB image
        vImages[0].Map.SetProperty( "CameraTime", m_ImageMD.Timestamp() );
        memcpy(vImages[0].Image.data, m_ImageMD.RGB24Data(), m_ImageMD.DataSize() );
    }

    if( m_bGetDepth ) {
        m_DepthNode.GetMetaData(m_DepthMD);

        // image 0 or 1 is depth image
        if( m_bGetRGB == false ) {
            vImages[0].Map.SetProperty( "CameraTime", m_DepthMD.Timestamp() );
            memcpy(vImages[0].Image.data, m_DepthMD.Data(), m_DepthMD.DataSize() );
        } else {
            vImages[1].Map.SetProperty( "CameraTime", m_DepthMD.Timestamp() );
            memcpy(vImages[1].Image.data, m_DepthMD.Data(), m_DepthMD.DataSize() );
        }
    }

    return true;
}
