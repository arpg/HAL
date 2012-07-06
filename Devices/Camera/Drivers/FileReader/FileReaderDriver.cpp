
#include "FileReaderDriver.h"
#include <Mvlpp/Utils.h>  // for FindFiles and PrintError
#include <boost/format.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "RPG/Devices/Camera/Drivers/Dvi2Pci/SDK/includes/s_fio.h"	// for imread()

//using namespace boost;
using namespace std;

///////////////////////////////////////////////////////////////////////////////
FileReaderDriver::FileReaderDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
FileReaderDriver::~FileReaderDriver()
{
	std::cout << "CALLING DESTRUCTOR..." << std::endl;
	m_CaptureThread->interrupt();
	m_CaptureThread->join();
}

///////////////////////////////////////////////////////////////////////////////
bool FileReaderDriver::Capture( std::vector<rpg::ImageWrapper>& vImages )
{

    // allocate images if necessary
    if( vImages.size() != m_nNumChannels ){
        vImages.resize( m_nNumChannels );
    }

	double dPctgFilled =  m_dBufferFilled/m_nBufferSize; 
	
	//while(m_vBufferFree[m_nNextCapture])
	while(m_vBufferFree[m_nNextCapture] || dPctgFilled < 0.5)
	{
		// cycle until next frame is available
		dPctgFilled =  m_dBufferFilled/m_nBufferSize; 
	}
	
	
	// allocate images if necessary
	if( vImages.size() != m_nNumChannels )
		vImages.resize( m_nNumChannels );
		
	// now fetch the next set of images from buffer
	for( unsigned int ii = 0; ii < m_nNumChannels; ii++ )	
		vImages[ii].Image = m_vImageBuffer[m_nNextCapture][ii].Image.clone();
		
	m_vBufferFree[m_nNextCapture] = true;
	m_nNextCapture				  = (m_nNextCapture+1) % m_nBufferSize;
	
	m_dBufferFilled -= 1.0;
	
    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool FileReaderDriver::Init()
{
	//std::cerr << "SlamThread: FileReader Init " << std::endl;

    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    m_nNumChannels       = m_pPropertyMap->GetProperty<unsigned int>( "NumChannels", 0 );
    m_nBufferSize        = m_pPropertyMap->GetProperty<unsigned int>( "BufferSize", 30 );
    m_nStartFrame        = m_pPropertyMap->GetProperty<unsigned int>( "StartFrame",  0 );
    m_nCurrentImageIndex = m_nStartFrame;
	
	//std::cerr << "start frame: " << m_nCurrentImageIndex << std::endl;

    m_vFileList.resize( m_nNumChannels );
    
    for( unsigned int ii = 0; ii < m_nNumChannels; ii++ ) {
		//std::cerr << "SlamThread: Finding files channel " << ii << std::endl;
        std::string sChannelName  = (boost::format("Channel-%d")%ii).str();
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
    
	//std::cerr << "SlamThread: Done reading filenames "  << std::endl;

    // make sure each channel has the same number of images
    m_nNumImages = m_vFileList[0].size();
    for( unsigned int ii = 1; ii < m_nNumChannels; ii++ ){
        if( m_vFileList[ii].size() != m_nNumImages ){
            mvl::PrintError( "ERROR: uneven number of files\n" );
            exit(1);
        }
    }
    
	//std::cerr << "SlamThread: Done checking equal num images"  << std::endl;

	//std::cerr << "SlamThread: Filling buffer: "  << m_nBufferSize << std::endl;
	m_dBufferFilled =  0;
	
	// fill image buffer
    m_vImageBuffer.resize(m_nBufferSize);
    for (unsigned int ii=0; ii < m_nBufferSize; ii++) {
		//std::cerr << "SlamThread: init reading "  << std::endl;
        _Read(m_vImageBuffer[ii]);
		//std::cerr << "SlamThread: Finished reading "  << std::endl;
		m_vBufferFree.push_back(false);
		//std::cerr << "SlamThread: push back "  << std::endl;

    }
 
	m_nNextCapture  =  0;
	m_nNextRead	    =  0;
	//std::cerr << "SlamThread: Start capture thread"  << std::endl;

	
//    boost::thread captureThread(boost::bind(&FileReaderDriver::_ThreadCaptureFunc,this)); 
//    boost::thread captureThread( _ThreadCaptureFunc, this );
    m_CaptureThread = new boost::thread( &_ThreadCaptureFunc, this );
    
    return true;
}

///////////////////////////////////////////////////////////////////////////////
void FileReaderDriver::_ThreadCaptureFunc( FileReaderDriver* pFR )
{
    while(1){
        
		try {
			boost::this_thread::interruption_point();
			if(pFR->m_vBufferFree[pFR->m_nNextRead])
			{   
				pFR->_Read(pFR->m_vImageBuffer[pFR->m_nNextRead]);
				pFR->m_vBufferFree[pFR->m_nNextRead] = false;
				pFR->m_nNextRead = (pFR->m_nNextRead+1) % pFR->m_nBufferSize; 
			}
		} catch( boost::thread_interrupted& interruption ) {
			std::cout << "!!!!!! THREAD INTERRUPTED !!!!!!" << std::endl;
			break;
		}
    }
}

///////////////////////////////////////////////////////////////////////////////
void FileReaderDriver::_Read( std::vector<rpg::ImageWrapper>& vImages)
{
	//std::cerr << "	+ SlamThread: _Read "  << m_nNumChannels << std::endl;

    // allocate images if necessary
    if( vImages.size() != m_nNumChannels ){
        vImages.resize( m_nNumChannels );
    }
    
	// loop over if we finished our files!
	if( m_nCurrentImageIndex == m_nNumImages ) {
        m_nCurrentImageIndex = m_nStartFrame;
	}
    
    // TODO: this is kinda lame and insecure, change eventually
    char imgFile[400];
    
    // now fetch the next set of images
	//cout << "Reading image pair" << endl;
	//double dTimeRead;
	
    for( unsigned int ii = 0; ii < m_nNumChannels; ii++ ) {
		//cout  << m_vFileList[ii][m_nCurrentImageIndex] << endl;
		
		sprintf( imgFile, "%s", m_vFileList[ii][m_nCurrentImageIndex].c_str() );
        
		//std::cerr << "		+ reading: "  << imgFile << std::endl;

		// TODO: this only reads grayscale '0'.. not sure if we need more than that tho
		//dTimeRead = mvl::Tic();
		vImages[ii].Image = cv::imread( imgFile, 0);
		//cout << "Read time: " << mvl::TocMS(dTimeRead) << endl;
		//std::cerr << " [done] "  << std::endl;

    }
    m_nCurrentImageIndex++;
    //return true;
	//std::cerr <<  "		imgIdx: "   << m_nCurrentImageIndex << std::endl;

	m_dBufferFilled += 1.0;
}

