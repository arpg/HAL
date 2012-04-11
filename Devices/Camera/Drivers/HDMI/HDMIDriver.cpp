
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

    // allocate images if necessary
    if( vImages.size() != m_nNumImages ){
        vImages.resize( m_nNumImages );
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


    result = m_pDeckLinkInput->EnableVideoInput(selectedDisplayMode, pixelFormat, inputFlags);
    if(result != S_OK)
    {
		fprintf(stderr, "Failed to enable video input. Is another application using the card?\n");
        return false;
    }



    return true;
}
