/*
   \file SimCamDriver.cpp
 */

#include "SimCamDriver.h"

#include <opencv/cv.h>	// for Mat structure
#include <stdio.h>

#include <Mvlpp/Mvl.h>
#include <Mvlpp/Utils.h>

//#include <Client.h>

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
SimCamDriver::SimCamDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
SimCamDriver::~SimCamDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
bool SimCamDriver::Capture( std::vector<cv::Mat>& vImages )
{

    // allocate images if necessary
//    if( vImages.size() != m_nNumCams ){
//        vImages.resize( m_nNumCams );
//        // and setup empty images
//        for(unsigned int ii = 0; ii < m_nNumCams; ii++ ) {
////            vImages[ii] = cv::Mat( m_nViewportHeight, m_nViewportWidth, CV_8UC1 );
//              vImages[ii] = cv::Mat( m_nViewportHeight, m_nViewportWidth, CV_8UC3 );
//        }
//        // for depth info
//        vImages[ m_nNumCams ] = cv::Mat( m_nViewportHeight, m_nViewportWidth, CV_8UC1 );
//    }


    // get texture

    // convert to grayscale

    // save depth in matrix

    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool SimCamDriver::Init()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    // read property map
    m_nNumCams = m_pPropertyMap->GetProperty<unsigned int>( "NumCams", 1 );

    // read property map
    std::string sCamModFile = m_pPropertyMap->GetProperty( "CamModFile", "" );

	std::vector< std::string >  vFileList;

	if( mvl::FindFiles( sCamModFile, vFileList ) == false ) {
		std::cout << "No files found from regexp!" << std::endl;
		exit(1);
	}

	for( int ii = 0; ii < m_nNumCams; ii++ ) {
		mvl::CameraModel CamModel( vFileList[ii] );

		m_vIntrinsics.push_back( CamModel.K() );
		m_vExtrinsics.push_back( CamModel.GetPose() );
	}

	mvl::Print( m_vIntrinsics[0], "Left Cam Model" );
	mvl::Print( m_vIntrinsics[1], "Right Cam Model" );
	mvl::Print( m_vExtrinsics[0], "Left Sensor Pose" );
	mvl::Print( m_vExtrinsics[1], "Right Sensor Pose" );

	// get client pointer from property map
	// SimClient* pSimClient;
	// pSimClient = m_pPropertyMap->GetProperty<SimClient*>( "SimClientPtr", NULL );
	// add cameraman
	// pSimClient->AddCameraMan();
	// store cameraman pointer in member variable

    return true;
}
