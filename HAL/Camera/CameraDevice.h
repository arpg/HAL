#pragma once

#include <PbMsgs/Image.h>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Devices/DeviceRegistry.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

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
    ~Camera()
    {
        Clear();
    }

    ///////////////////////////////////////////////////////////////
    inline void Reset()
    {
        Clear();
        m_cam = DeviceRegistry<CameraDriverInterface>::I().Create(m_uri);
    }

    ///////////////////////////////////////////////////////////////
    inline void Clear()
    {
//        DeviceRegistry<CameraDriverInterface>::Destroy(m_cam);
        m_cam = nullptr;
    }

    ///////////////////////////////////////////////////////////////
    bool Capture( pb::CameraMsg& Images )
    {
        Images.Clear();
        return m_cam->Capture(Images);
    }

    ///////////////////////////////////////////////////////////////
    bool Capture( pb::ImageArray& Images )
    {
        return Capture( Images.ref() );
    }

protected:
    hal::Uri m_uri;
    std::shared_ptr<CameraDriverInterface> m_cam;
};

}
