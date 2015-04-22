#pragma once


#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Camera.pb.h>

#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <gui/SurfaceTexture.h>
#include <gui/SurfaceTextureClient.h>
#pragma GCC diagnostic pop

#include "CameraListener.h"

namespace hal
{

class AndroidDriver : public CameraDriverInterface
{
public:
    AndroidDriver();
    ~AndroidDriver();

    bool Capture( hal::CameraMsg& vImages );

    size_t NumChannels() const;
    size_t Width( size_t /*idx*/ = 0 ) const;
    size_t Height( size_t /*idx*/ = 0 ) const;

    inline std::shared_ptr<CameraDriverInterface> GetInputDevice() {
        return std::shared_ptr<CameraDriverInterface>();
    }

protected:
    android::sp<android::Camera>                m_Cam;
    android::sp<HALCameraListener>              m_CamListener;
    android::sp<android::SurfaceTexture>        m_SurfaceTexture ;
    android::sp<android::SurfaceTextureClient>  m_SurfaceTextureClient ;
    GLuint                                      m_Texture;
    size_t                                      m_nWidth;
    size_t                                      m_nHeight;
};

}
