/*
   \file NodeCamDriver.cpp
 */

#include "NodeCamDriver.h"

#include <opencv/cv.h>	// for Mat structure
#include <stdio.h>

#include <Mvlpp/Mvl.h>
#include <Mvlpp/Utils.h>


///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
NodeCamDriver::NodeCamDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
//  Releases the cameras and exits
NodeCamDriver::~NodeCamDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
bool NodeCamDriver::Capture( std::vector<cv::Mat>& vImages )
{

    // clear image vector
	vImages.clear();

	// allocate ZeroMQ message
	zmq::message_t ZmqMsg;

	// read from ZMQ
	if( m_pSocket->recv( &ZmqMsg ) == false ) {
		// nothing to read
		return false;
	}

	if ( ZmqMsg.size() == 0 ) {
		return false;
	}

	// message type is of format:
	// NumImages|Img1Width|Img1Height|Img1Format|Img1Data|Img2Width|...

	// pointer to message data
	char* MsgPtr = (char*)ZmqMsg.data();

	// get number of cameras
	int NumImgs;
	std::memcpy( &NumImgs, MsgPtr, sizeof(NumImgs) );
	MsgPtr += sizeof(NumImgs);

	// resize output vector
	vImages.resize(NumImgs);

	for( int ii = 0; ii < NumImgs; ii++ ) {

		int ImgWidth;
		std::memcpy( &ImgWidth, MsgPtr, sizeof(ImgWidth) );
		MsgPtr += sizeof(ImgWidth);
		int ImgHeight;
		std::memcpy( &ImgHeight, MsgPtr, sizeof(ImgHeight) );
		MsgPtr += sizeof(ImgHeight);
		int ImgType;
		std::memcpy( &ImgType, MsgPtr, sizeof(ImgType) );
		MsgPtr += sizeof(ImgType);

		// decode image type from OpenCV type
		unsigned int ImgDepth = ((ImgType & 0x0111) / 2 ) + 1;
		unsigned int ImgChannels = (ImgType >> 3) + 1;

		vImages[ii] = cv::Mat( ImgHeight, ImgWidth, ImgType, (void*)MsgPtr );
		MsgPtr += ImgHeight * ImgWidth * ImgDepth * ImgChannels;
	}


    // save depth in matrix
	return true;
}


///////////////////////////////////////////////////////////////////////////////
bool NodeCamDriver::Init()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    // read property map
	m_sHost = m_pPropertyMap->GetProperty( "Host", "localhost:5556" );

	// init ZMQ context
	m_pContext = new zmq::context_t(1);

	// create SUB socket
	m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_SUB );

	// lets connect using the socket
	try {
		m_pSocket->setsockopt( ZMQ_SUBSCRIBE, NULL, 0 );
		m_pSocket->connect( ("tcp://" + m_sHost).c_str() );
	} catch( zmq::error_t error ) {
		// oops, an error occurred lets rollback
		delete m_pSocket;
		return false;
	}
	return true;
}
