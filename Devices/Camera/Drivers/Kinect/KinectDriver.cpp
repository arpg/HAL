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
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    std::string ConfigFile = m_pPropertyMap->GetProperty( "ConfigFile", "");

    // for now we will init context through a config file specified
    // by the property map.. in the future this will be removed
   	XnStatus rc;

	EnumerationErrors errors;
    rc = m_Context.InitFromXmlFile( ConfigFile.c_str(), m_ScriptNode, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		return false;
	}

	rc = m_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, m_DepthNode);
	if (rc != XN_STATUS_OK)
	{
		printf("No depth node exists! Check your XML.");
		return false;
	}

	rc = m_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, m_ImageNode);
	if (rc != XN_STATUS_OK)
	{
		printf("No image node exists! Check your XML.");
		return false;
	}

    // read once to get some info on resolution and image format
	m_DepthNode.GetMetaData(m_DepthMD);
	m_ImageNode.GetMetaData(m_ImageMD);

	// Hybrid mode isn't supported in this sample
	if (m_ImageMD.FullXRes() != m_DepthMD.FullXRes() || m_ImageMD.FullYRes() != m_DepthMD.FullYRes())
	{
		printf ("The device depth and image resolution must be equal!\n");
		return false;
	}

	// RGB is the only image format supported.
	if (m_ImageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		printf("The device image format must be RGB24\n");
		return false;
	}

    // "mirror" the image.. not sure if we should do that?
    // perhaps pass through PropertyMap
    m_Context.SetGlobalMirror(true);

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

	m_DepthNode.GetMetaData(m_DepthMD);
	m_ImageNode.GetMetaData(m_ImageMD);

	const XnDepthPixel* pDepth = m_DepthMD.Data();
	const XnUInt8* pImage = m_ImageMD.Data();

    // prepare return images
    vImages.clear();
    vImages.resize(2);

    //----------------------------------------------------------------------------
    // image 0 is RGB image
    vImages[0] = cv::Mat( 480, 640, CV_8UC3 );

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
    vImages[1] = cv::Mat( 480, 640, CV_8UC3 );

    float DepthHist[MAX_DEPTH];

    // Calculate the accumulative histogram (the yellow display...)
	xnOSMemSet(DepthHist, 0, MAX_DEPTH*sizeof(float));

	unsigned int nNumberOfPoints = 0;
	for (unsigned int y = 0; y < m_DepthMD.YRes(); ++y)
	{
		for (unsigned int x = 0; x < m_DepthMD.XRes(); ++x, ++pDepth)
		{
			if (*pDepth != 0)
			{
				DepthHist[*pDepth]++;
				nNumberOfPoints++;
			}
		}
	}
	for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		DepthHist[nIndex] += DepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			DepthHist[nIndex] = (unsigned int)(256 * (1.0f - (DepthHist[nIndex] / nNumberOfPoints)));
		}
	}


    const XnDepthPixel* pDepthRow = m_DepthMD.Data();

    for (unsigned int y = 0; y < m_DepthMD.YRes(); ++y)
    {
        const XnDepthPixel* pDepth = pDepthRow;

        for (unsigned int x = 0; x < m_DepthMD.XRes(); ++x, ++pDepth)
        {
            if (*pDepth != 0)
            {
                int nHistValue = DepthHist[*pDepth];
                unsigned int idx = x * 3;
                vImages[1].at<unsigned char>(y,idx) = 0;
                vImages[1].at<unsigned char>(y,idx+1) = nHistValue;
                vImages[1].at<unsigned char>(y,idx+2) = nHistValue;
            }
        }

        pDepthRow += m_DepthMD.XRes();
    }

    return true;
}