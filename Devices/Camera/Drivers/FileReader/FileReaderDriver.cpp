
#include "FileReaderDriver.h"
#include <Mvlpp/Utils.h>  // for FindFiles and PrintError
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include "opencv2/highgui/highgui.hpp"	// for imread()

using namespace boost;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
FileReaderDriver::FileReaderDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
FileReaderDriver::~FileReaderDriver()
{

}

///////////////////////////////////////////////////////////////////////////////
bool FileReaderDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{

    // allocate images if necessary
    if( vImages.size() != m_nNumChannels ){
        vImages.resize( m_nNumChannels );
    }

	while(m_vBufferFree[m_nNextCapture])
	{
		// cycle until next frame is available
	}
	
	// allocate images if necessary
	if( vImages.size() != m_nNumChannels )
		vImages.resize( m_nNumChannels );
		
	// now fetch the next set of images from buffer
	for( unsigned int ii = 0; ii < m_nNumChannels; ii++ )	
		vImages[ii].Image = m_vImageBuffer[m_nNextCapture][ii].Image.clone();
		
	m_vBufferFree[m_nNextCapture] = true;
	m_nNextCapture				  = (m_nNextCapture+1) % m_nBufferSize;
	
	
    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool FileReaderDriver::Init()
{
    
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    m_nNumChannels       = m_pPropertyMap->GetProperty<unsigned int>( "NumChannels", 0 );
    m_nBufferSize        = m_pPropertyMap->GetProperty<unsigned int>( "BufferSize", 20 );
    m_nStartFrame        = m_pPropertyMap->GetProperty<unsigned int>( "StartFrame",  0 );
    m_nCurrentImageIndex = m_nStartFrame;

    m_vFileList.resize( m_nNumChannels );
    
    for( unsigned int ii = 0; ii < m_nNumChannels; ii++ ) {
        std::string sChannelName  = (format("Channel-%d")%ii).str();
        std::string sChannelRegex = m_pPropertyMap->GetProperty( sChannelName, "");

        // Get data path 
        std::string sChannelPath = m_pPropertyMap->GetProperty( "DataSourceDir", "");

        // Now generate the list of files for each channel
        std::vector< std::string>& vFiles = m_vFileList[ii];

        if(mvl::FindFiles(sChannelPath, sChannelRegex, vFiles) == false){
        //if( mvl::FindFiles( sChannelRegex, vFiles ) == false ) {
            mvl::PrintError( "ERROR: Not files found from regexp\n" );
            exit(1);
        }
    }
    
    // make sure each channel has the same number of images
    m_nNumImages = m_vFileList[0].size();
    for( unsigned int ii = 1; ii < m_nNumChannels; ii++ ){
        if( m_vFileList[ii].size() != m_nNumImages ){
            mvl::PrintError( "ERROR: uneven number of files\n" );
            exit(1);
        }
    }
    
    // fill image buffer
    m_vImageBuffer.resize(m_nBufferSize);
    for (unsigned int ii=0; ii < m_nBufferSize; ii++) {
        _Read(m_vImageBuffer[ii]);
		m_vBufferFree.push_back(false);
    }

	m_nNextCapture =  0;
	m_nNextRead	   =  0;
	
    boost::thread captureThread(boost::bind(&FileReaderDriver::_ThreadCaptureFunc,this));
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////
void FileReaderDriver::_ThreadCaptureFunc()
{
    while(1){
        
		if(m_vBufferFree[m_nNextRead])
		{
			_Read(m_vImageBuffer[m_nNextRead]);
			m_vBufferFree[m_nNextRead] = false;
			m_nNextRead				   = (m_nNextRead+1) % m_nBufferSize; 
		}
    }
}

///////////////////////////////////////////////////////////////////////////////
void FileReaderDriver::_Read( std::vector<rpg::ImageWrapper>& vImages)
{
    // allocate images if necessary
    if( vImages.size() != m_nNumChannels ){
        vImages.resize( m_nNumChannels );
    }
    
	// loop over if we finished our files!
	if( m_nCurrentImageIndex == m_nNumImages ) {
        m_nCurrentImageIndex = m_nStartFrame;
	}
    
    // TODO: this is kinda lame and insecure, change eventually
    char imgFile[100];
    
    // now fetch the next set of images
	//cout << "Reading image pair" << endl;
    for( unsigned int ii = 0; ii < m_nNumChannels; ii++ ) {
		//cout  << m_vFileList[ii][m_nCurrentImageIndex] << endl;
		
		sprintf( imgFile, "%s", m_vFileList[ii][m_nCurrentImageIndex].c_str() );
        
		// TODO: this only reads grayscale '0'.. not sure if we need more than that tho
		vImages[ii].Image = cv::imread( imgFile, 0);
		
    }
    m_nCurrentImageIndex++;
    //return true;
}

