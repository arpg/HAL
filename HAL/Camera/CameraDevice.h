#pragma once

#include <PbMsgs/Image.h>

#include <HAL/Uri.h>
#include <HAL/DeviceRegistry.h>
#include <HAL/Camera/CameraDriverInterface.h>

namespace hal {

///////////////////////////////////////////////////////////////////////////////
// Generic camera device
class Camera : public CameraDriverInterface
{
public:
    ///////////////////////////////////////////////////////////////
    Camera()
    {
    }
    
    ///////////////////////////////////////////////////////////////
    Camera(const std::string& uri)
        : m_uri(uri)
    {
        m_cam = DeviceRegistry<CameraDriverInterface>::I().Create(m_uri);
    }    
    
    ///////////////////////////////////////////////////////////////
    inline void Reset()
    {
        Clear();
        m_cam = DeviceRegistry<CameraDriverInterface>::I().Create(m_uri);
    }   
    
    ///////////////////////////////////////////////////////////////
    ~Camera()
    {
        Clear();
    }
    
    ///////////////////////////////////////////////////////////////
    bool Capture( pb::CameraMsg& Images )
    {
        return m_cam->Capture(Images);
    }
    
    ///////////////////////////////////////////////////////////////
    bool Capture( pb::ImageArray& Images )
    {
        const bool ret = Capture( Images.ref() );
        Images.SelfUpdate();
        return ret;
    }
    
protected:
    hal::Uri m_uri;
    std::shared_ptr<CameraDriverInterface> m_cam;    
};

}
