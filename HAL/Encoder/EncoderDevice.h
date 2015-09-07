#pragma once

#include <HAL/Devices/SharedLoad.h>

#include <HAL/Encoder/EncoderDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {

///////////////////////////////////////////////////////////////////////////////
// Generic Encoder device
class Encoder : public EncoderDriverInterface
{
    public:
        ///////////////////////////////////////////////////////////////
        Encoder()
        {
        }

        ///////////////////////////////////////////////////////////////
        Encoder(const std::string& uri)
            : m_URI(uri)
        {
       //   m_Encoder = DeviceRegistry<EncoderDriverInterface>::Instance().Create(m_URI, "Encoder");
          m_Encoder = DeviceRegistry<EncoderDriverInterface>::Instance().Create(m_URI);
        }

        ///////////////////////////////////////////////////////////////
        ~Encoder()
        {
            Clear();
        }

        ///////////////////////////////////////////////////////////////
        void Clear()
        {
            m_Encoder = nullptr;
        }

        ///////////////////////////////////////////////////////////////
        void RegisterEncoderDataCallback(EncoderDriverDataCallback callback)
        {
            if( m_Encoder ){
                m_Encoder->RegisterEncoderDataCallback( callback );
            }else{
                std::cerr << "ERROR: no driver initialized!\n";
            }
            return;
        }

        ///////////////////////////////////////////////////////////////
        std::string GetDeviceProperty(const std::string& sProperty)
        {
            return m_Encoder->GetDeviceProperty(sProperty);
        }


protected:
    hal::Uri                                m_URI;
    std::shared_ptr<EncoderDriverInterface> m_Encoder;

};

} /* namespace */
