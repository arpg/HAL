
#include "HDMIDriver.h"


#include "opencv/cv.h"	// for Mat structure


///////////////////////////////////////////////////////////////////////////////
HDMIDriver::HDMIDriver()
{
}

///////////////////////////////////////////////////////////////////////////////
HDMIDriver::~HDMIDriver()
{

}

///////////////////////////////////////////////////////////////////////////////
bool HDMIDriver::Capture( std::vector<cv::Mat>& vImages )
{
    // if there are any matrices, delete them 
    vImages.clear();
    
    char *buffer;
    int length;
    // while there are no frames, block the calls
    while(m_pDelegate->GetFrame(buffer, length) == false ){ }
    
    
    // allocate images if necessary
    if( vImages.size() != m_nNumImages ){
        vImages.resize( m_nNumImages );
    }
    
    // now copy the images from the delegate
    for (int ii = 0; ii < m_nNumImages; ii++) {
        vImages[ii] = cv::Mat(m_nImageHeight,m_nImageWidth, CV_8UC3, buffer);
        //advance the pointer forward
        buffer += m_nImageHeight*m_nImageWidth*4;
    }
    return true;
}


///////////////////////////////////////////////////////////////////////////////
bool HDMIDriver::Init()
{
    assert(m_pPropertyMap);
    m_pPropertyMap->PrintPropertyMap();

    m_nNumImages = m_pPropertyMap->GetProperty<int>( "NumImages", 1 );
    m_nImageWidth = m_pPropertyMap->GetProperty<int>( "ImageWidth", 512 );
    m_nImageHeight = m_pPropertyMap->GetProperty<int>( "ImageHeight", 384 );
    int bufferCount = m_pPropertyMap->GetProperty<int>("BufferCount",5);

	m_pIterator = CreateDeckLinkIteratorInstance();

	HRESULT						result;

	// Connect to the first DeckLink instance
	result = m_pIterator->Next( &m_pDeckLink );
	if (result != S_OK)
	{
		fprintf(stderr, "No DeckLink PCI cards found.\n");
		return false;
	}

	if (m_pDeckLink->QueryInterface(IID_IDeckLinkInput, (void**)&m_pDeckLinkInput) != S_OK) {
		return false;
	}

	BMDVideoInputFlags			inputFlags = 0;
	BMDDisplayMode				selectedDisplayMode = bmdModeHD720p60;
	BMDPixelFormat				pixelFormat = bmdFormat8BitYUV;

    // set up the callback delegate
    
    m_pDelegate = new CaptureDelegate(bufferCount,m_nNumImages,m_nImageWidth,m_nImageHeight);
    m_pDeckLinkInput->SetCallback( m_pDelegate );

    result = m_pDeckLinkInput->EnableVideoInput(selectedDisplayMode, pixelFormat, inputFlags);
    if(result != S_OK)
    {
		fprintf(stderr, "Failed to enable video input. Is another application using the card?\n");
        return false;
    }
    
    
    // start stream
    result = m_pDeckLinkInput->StartStreams();
    if(result != S_OK) {
        return false;
    }

    return true;
}
