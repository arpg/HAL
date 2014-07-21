#include "OpenNIDriver.h"

#include <HAL/Devices/DeviceException.h>
#include <HAL/Utils/TicToc.h>

#include <iostream>

#define MAX_DEPTH 10000

using namespace xn;
using namespace hal;

#define CHECK_XN_RETURN(rc) { \
    if (rc != XN_STATUS_OK) { \
        XnChar strError[1024]; \
        EnumerationErrors errors; \
        errors.ToString(strError, 1024); \
        throw DeviceException(strError); \
    } \
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OpenNIDriver::OpenNIDriver(
        unsigned int            nWidth,
        unsigned int            nHeight,
        unsigned int            nFPS,
        bool                    bCaptureRGB,
        bool                    bCaptureDepth,
        bool                    bCaptureIR,
        bool                    bAlignDepth
    )
{
    XnMapOutputMode MapMode;
    MapMode.nFPS = nFPS;

    m_nImgWidth = nWidth;
    m_nImgHeight = nHeight;
    MapMode.nXRes = nWidth;
    MapMode.nYRes = nHeight;

    XnStatus rc;

    rc = m_Context.Init();
    CHECK_XN_RETURN(rc);

    // get device ids
    {
      NodeInfoList DeviceNodeList;
      rc = m_Context.EnumerateProductionTrees( XN_NODE_TYPE_DEVICE, NULL, DeviceNodeList, 0 );
      CHECK_XN_RETURN(rc);

      for(NodeInfoList::Iterator it = DeviceNodeList.Begin(); it != DeviceNodeList.End(); ++it) {
        NodeInfo node = *it;

        //idVendor/idProduct@BusID/DeviceId
        // idVendor, idProduct, DeviceId: 4 hex digits
        std::string info(node.GetCreationInfo());
        std::string::size_type first = info.find('@');
        long int serialno = 0;
        if (first != std::string::npos)
        {
          ++first; // first BusID character
          std::string::size_type mid = info.find('/', first);
          if (mid != std::string::npos)
          {
            std::string sBus, sDevice;
            sBus = info.substr(first, mid - first);
            sDevice = info.substr(mid + 1);
            long bus = std::stol(sBus);
            long device = std::stol(sDevice);

            // serialno 32 least significant bits: BusId, DeviceID
            serialno = (bus << 16) | device;
          }
        }
        m_SerialNos.push_back(serialno);
      }
    }

    if( bCaptureRGB ) {
        // Enumerate Image Generating nodes
        NodeInfoList ImageNodeList;
        rc = m_Context.EnumerateProductionTrees( XN_NODE_TYPE_IMAGE, NULL, ImageNodeList, 0 );
        CHECK_XN_RETURN(rc);

        for(NodeInfoList::Iterator it = ImageNodeList.Begin(); it != ImageNodeList.End(); ++it) {
            // create depth generator
            NodeInfo node = *it;

            xn::ImageGenerator imageGen;

            rc = m_Context.CreateProductionTree(node, imageGen);
            CHECK_XN_RETURN(rc);

            rc = node.GetInstance(imageGen);
            CHECK_XN_RETURN(rc);

            rc = imageGen.SetMapOutputMode(MapMode);

            m_ImageGenerators.push_back(imageGen);
        }
    }


    if( bCaptureDepth ) {
        // Enumerate Depth Generating nodes
        NodeInfoList DepthNodeList;
        rc = m_Context.EnumerateProductionTrees( XN_NODE_TYPE_DEPTH, NULL, DepthNodeList, 0 );
        CHECK_XN_RETURN(rc);

        unsigned int count = 0;
        for(NodeInfoList::Iterator it = DepthNodeList.Begin(); it != DepthNodeList.End(); ++it) {
            // create depth generator
            NodeInfo node = *it;

            xn::DepthGenerator depthGen;

            rc = m_Context.CreateProductionTree(node, depthGen);
            CHECK_XN_RETURN(rc);

            rc = node.GetInstance(depthGen);
            CHECK_XN_RETURN(rc);

            rc = depthGen.SetMapOutputMode(MapMode);
            CHECK_XN_RETURN(rc);

            // --- ALIGN DEPTH TO RGB IF REQUESTED ---
            if( bAlignDepth ) {
                if( depthGen.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT) ) {
                    AlternativeViewPointCapability avpx = depthGen.GetAlternativeViewPointCap();
                    rc = avpx.SetViewPoint( m_ImageGenerators[count]  );
                    CHECK_XN_RETURN(rc);
                }
                count++;
            }


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

            m_DepthBaselines.push_back( dBaselineRGBDepth );
            m_DepthFocalLengths.push_back( depth_focal_length_SXGA / 2 );

            // --- END GETTING PARAMS ---


            m_DepthGenerators.push_back(depthGen);
        }
    }


    if( bCaptureIR ) {
        // Enumerate IR Generating nodes
        NodeInfoList IRNodeList;
        rc = m_Context.EnumerateProductionTrees( XN_NODE_TYPE_IR, NULL, IRNodeList, 0 );
        CHECK_XN_RETURN(rc);

        for(NodeInfoList::Iterator it = IRNodeList.Begin(); it != IRNodeList.End(); ++it) {
            // create depth generator
            NodeInfo node = *it;

            xn::IRGenerator irGen;

            rc = m_Context.CreateProductionTree(node, irGen);
            CHECK_XN_RETURN(rc);

            rc = node.GetInstance(irGen);
            CHECK_XN_RETURN(rc);

            rc = irGen.SetMapOutputMode(MapMode);

            m_IRGenerators.push_back(irGen);
        }
    }

    rc = m_Context.StartGeneratingAll();
    CHECK_XN_RETURN(rc);
}

#undef CHECK_XN_RETURN

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
OpenNIDriver::~OpenNIDriver()
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool OpenNIDriver::Capture( pb::CameraMsg& vImages )
{
    XnStatus rc = XN_STATUS_OK;

    // Read a new frame
    rc = m_Context.WaitAndUpdateAll();
    const double systemTime = Tic();
    vImages.set_device_time( systemTime );

    if (rc != XN_STATUS_OK)
    {
        printf("Read failed: %s\n", xnGetStatusString(rc));
        return false;
    }

    // prepare return images

    for(unsigned int i=0; i<m_ImageGenerators.size(); ++i) {
        xn::ImageMetaData metaData;
        m_ImageGenerators[i].GetMetaData(metaData);
        pb::ImageMsg* pbImg = vImages.add_image();
        pbImg->set_timestamp( metaData.Timestamp() );
        pbImg->set_width( m_nImgWidth );
        pbImg->set_height( m_nImgHeight );
        pbImg->set_type(pb::PB_UNSIGNED_BYTE);
        pbImg->set_format(pb::PB_RGB);
        pbImg->set_data( metaData.RGB24Data(), metaData.DataSize() );
    }
    for(unsigned int i=0; i<m_DepthGenerators.size(); ++i) {
        xn::DepthMetaData metaData;
        m_DepthGenerators[i].GetMetaData(metaData);
        pb::ImageMsg* pbImg = vImages.add_image();
        pbImg->set_width( m_nImgWidth );
        pbImg->set_height( m_nImgHeight );
        pbImg->set_timestamp( metaData.Timestamp() );
        pbImg->set_data( metaData.Data(), metaData.DataSize() );
        pbImg->set_type(pb::PB_UNSIGNED_SHORT);
        pbImg->set_format(pb::PB_LUMINANCE);
        pb::ImageInfoMsg* pbImgInfo = pbImg->mutable_info();
        pbImgInfo->set_baseline( m_DepthBaselines[i] );
        pbImgInfo->set_focal_length( m_DepthFocalLengths[i] );
    }
    for(unsigned int i=0; i<m_IRGenerators.size(); ++i) {
        xn::IRMetaData metaData;
        m_IRGenerators[i].GetMetaData(metaData);
        pb::ImageMsg* pbImg = vImages.add_image();
        pbImg->set_width( m_nImgWidth );
        pbImg->set_height( m_nImgHeight );
        pbImg->set_timestamp( metaData.Timestamp() );
        pbImg->set_data( metaData.Data(), metaData.DataSize() );
        pbImg->set_type(pb::PB_UNSIGNED_SHORT);
        pbImg->set_format(pb::PB_LUMINANCE);
    }

    return true;
}

std::string OpenNIDriver::GetDeviceProperty(const std::string& sProperty)
{
    // TODO add property with suffix of camera (ie. DepthBaseline0, DepthBaseline1)
    // and return correct vector info
    if(sProperty == hal::DeviceDepthBaseline) {
        return std::to_string( m_DepthBaselines[0] );
    }
    if(sProperty == hal::DeviceDepthFocalLength) {
        return std::to_string( m_DepthFocalLengths[0] );
    }
    return std::string();
}

size_t OpenNIDriver::NumChannels() const
{
    return m_ImageGenerators.size() +
            m_DepthGenerators.size() +
            m_IRGenerators.size();
}
size_t OpenNIDriver::Width( size_t /*idx*/ ) const
{
    return m_nImgWidth;
}

size_t OpenNIDriver::Height( size_t /*idx*/ ) const
{
    return m_nImgHeight;
}
