/*
   \file KinectDriver.cpp
 */

#include "KinectDriver.h"

#include <boost/lexical_cast.hpp>

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

#define CHECK_XN_RETURN(rc) { \
    if (rc != XN_STATUS_OK) { \
        XnChar strError[1024]; \
        EnumerationErrors errors; \
        errors.ToString(strError, 1024); \
        printf("%s\n", strError); \
        return false; \
    } \
}

///////////////////////////////////////////////////////////////////////////////
//
bool KinectDriver::Init()
{
    std::cout << "KinectDriver::Init" << std::endl;
    bool bGetImage = m_pPropertyMap->GetProperty( "GetRGB", true );
    bool bGetDepth = m_pPropertyMap->GetProperty( "GetDepth", true );
    bool bGetIR    = m_pPropertyMap->GetProperty( "GetIr", false );

    XnMapOutputMode MapMode;
    MapMode.nXRes = XN_VGA_X_RES;
    MapMode.nYRes = XN_VGA_Y_RES;
    MapMode.nFPS = 30;

    XnStatus rc;

    rc = m_Context.Init ();
    CHECK_XN_RETURN(rc);

    if( bGetDepth ) {
        // Enumerate Depth Generating nodes
        NodeInfoList DepthNodeList;
        rc = m_Context.EnumerateProductionTrees( XN_NODE_TYPE_DEPTH, NULL, DepthNodeList, 0 );
        CHECK_XN_RETURN(rc);

        for(NodeInfoList::Iterator it = DepthNodeList.Begin(); it != DepthNodeList.End(); ++it) {
            // create depth generator
            const NodeInfo& node = *it;

            xn::DepthGenerator depthGen;

            rc = m_Context.CreateProductionTree(const_cast<xn::NodeInfo&>(node));
            CHECK_XN_RETURN(rc);

            rc = node.GetInstance(depthGen);
            CHECK_XN_RETURN(rc);

            rc = depthGen.SetMapOutputMode(MapMode);
            CHECK_XN_RETURN(rc);

            // --- GET CAMERA PARAMS ---
            // Details can be found in XnStreamParams.h

            // focal length in mm
            XnUInt64 depth_focal_length_SXGA_mm;   //in mm
            depthGen.GetIntProperty("ZPD", depth_focal_length_SXGA_mm);

            // pixel size in mm
            XnDouble pixelSize;
            depthGen.GetRealProperty( "ZPPS", pixelSize );  // in mm

            // focal length in pixels
            double depth_focal_length_SXGA = depth_focal_length_SXGA_mm / pixelSize;

            XnDouble dBaselineRGBDepth;
            depthGen.GetRealProperty( "DCRCDIS", dBaselineRGBDepth );

            const int camn = m_DepthGenerators.size();
            m_pPropertyMap->SetProperty( "Depth" +  boost::lexical_cast<std::string>(camn) + "FocalLength", depth_focal_length_SXGA / 2 );
            m_pPropertyMap->SetProperty( "Depth" +  boost::lexical_cast<std::string>(camn) + "Baseline", dBaselineRGBDepth );

            m_DepthGenerators.push_back(depthGen);
        }
    }

    if( bGetImage ) {
        // Enumerate Image Generating nodes
        NodeInfoList ImageNodeList;
        rc = m_Context.EnumerateProductionTrees( XN_NODE_TYPE_IMAGE, NULL, ImageNodeList, 0 );
        CHECK_XN_RETURN(rc);

        for(NodeInfoList::Iterator it = ImageNodeList.Begin(); it != ImageNodeList.End(); ++it) {
            // create depth generator
            const NodeInfo& node = *it;

            xn::ImageGenerator imageGen;

            rc = m_Context.CreateProductionTree(const_cast<xn::NodeInfo&>(node));
            CHECK_XN_RETURN(rc);

            rc = node.GetInstance(imageGen);
            CHECK_XN_RETURN(rc);

            rc = imageGen.SetMapOutputMode(MapMode);

            m_ImageGenerators.push_back(imageGen);
        }
    }

    if( bGetIR ) {
        // Enumerate IR Generating nodes
        NodeInfoList IRNodeList;
        rc = m_Context.EnumerateProductionTrees( XN_NODE_TYPE_IR, NULL, IRNodeList, 0 );
        CHECK_XN_RETURN(rc);

        for(NodeInfoList::Iterator it = IRNodeList.Begin(); it != IRNodeList.End(); ++it) {
            // create depth generator
            const NodeInfo& node = *it;

            xn::IRGenerator irGen;

            rc = m_Context.CreateProductionTree(const_cast<xn::NodeInfo&>(node));
            CHECK_XN_RETURN(rc);

            rc = node.GetInstance(irGen);
            CHECK_XN_RETURN(rc);

            rc = irGen.SetMapOutputMode(MapMode);

            m_IRGenerators.push_back(irGen);
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
    CHECK_XN_RETURN(rc);

    return true;
}

#undef CHECK_XN_RETURN

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
    const unsigned int nNumImgs = m_DepthGenerators.size() + m_ImageGenerators.size() + m_IRGenerators.size();
    if( vImages.size() != nNumImgs )
    {
        vImages.resize( nNumImgs );

        int n = 0;
        for(unsigned int i=0; i<m_ImageGenerators.size(); ++i) {
            vImages[n++].Image = cv::Mat( 480, 640, CV_8UC3 );
        }
        for(unsigned int i=0; i<m_DepthGenerators.size(); ++i) {
            vImages[n++].Image = cv::Mat( 480, 640, CV_16UC1 );
        }
        for(unsigned int i=0; i<m_IRGenerators.size(); ++i) {
            vImages[n++].Image = cv::Mat( 480, 640, CV_16UC1 );
        }
    }

    int n = 0;
    for(unsigned int i=0; i<m_ImageGenerators.size(); ++i) {
        xn::ImageMetaData metaData;
        m_ImageGenerators[i].GetMetaData(metaData);
        vImages[n].Map.SetProperty( "CameraTime", metaData.Timestamp() );
        memcpy(vImages[n++].Image.data, metaData.RGB24Data(), metaData.DataSize() );
    }
    for(unsigned int i=0; i<m_DepthGenerators.size(); ++i) {
        xn::DepthMetaData metaData;
        m_DepthGenerators[i].GetMetaData(metaData);
        vImages[n].Map.SetProperty( "CameraTime", metaData.Timestamp() );
        memcpy(vImages[n++].Image.data, metaData.Data(), metaData.DataSize() );
    }
    for(unsigned int i=0; i<m_IRGenerators.size(); ++i) {
        xn::IRMetaData metaData;
        m_IRGenerators[i].GetMetaData(metaData);
        vImages[n].Map.SetProperty( "CameraTime", metaData.Timestamp() );
        memcpy(vImages[n++].Image.data, metaData.Data(), metaData.DataSize() );
    }

    return true;
}
