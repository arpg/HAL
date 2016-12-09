#include "AutoExposureDriver.h"
#include <HAL/Devices/DeviceException.h>

namespace hal
{

AutoExposureDriver::AutoExposureDriver(const int nTarget, std::shared_ptr<CameraDriverInterface> Input)
    : m_Input(Input), m_nTarget(nTarget),m_nExposure(1),m_fLastError(0)
{
    m_pUvcDriver = dynamic_cast<UvcDriver*>(Input.get());
    if(m_pUvcDriver == NULL) {
        throw DeviceException( "The input device to the autoexposure driver must be a uvc device." );
    }
}

bool AutoExposureDriver::Capture( hal::CameraMsg& vImages )
{
    m_pUvcDriver->SetExposure(m_nExposure);
    m_Input->Capture( vImages );

    if( vImages.image_size() > 1 ) {
        std::cerr << "error: Autoexposure is expecting 1 image but instead got " << vImages.image_size() << "." << std::endl;
        return false;
    }

    const hal::ImageMsg& InImg = vImages.image(0);

    if( InImg.type() != hal::PB_UNSIGNED_BYTE || InImg.format() != hal::PB_LUMINANCE){
        std::cerr << "error: Autoexposure currently only works with unsigned byte and luminance images." << std::endl;
    }

    float fMean = 0;
    const unsigned char* pData = (const unsigned char*)&InImg.data().front();
    for( int ii = 0, s = Width()*Height(); ii < s ; ii++ ) {
        fMean += *pData;
        pData++;
    }

    // calculate the mean intensity
    fMean /= Width()*Height();
    const float fError = m_nTarget - fMean;
    int nNewExposure = m_nExposure +  fError*0.2 + (fError-m_fLastError)*0.1;
    // TODO: Work out the valid maximum exposure (exposure set too high will cause flickering )
    nNewExposure = std::max(0,std::min(500,nNewExposure));
    m_fLastError = fError;

    if( nNewExposure != m_nExposure ){
        m_nExposure = nNewExposure;
    }

    return true;
}

std::string AutoExposureDriver::GetDeviceProperty(const std::string& sProperty)
{
    return m_Input->GetDeviceProperty(sProperty);
}
}
