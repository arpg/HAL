#pragma once

#include <HAL/Encoder/EncoderDriverInterface.h>
#include <HAL/Devices/DriverFactory.h>
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
          m_Encoder = DeviceDriverRegistry<EncoderDriverInterface>::Instance().Create(m_URI);
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

        /*
        ///////////////////////////////////////////////////////////////
        std::string GetProperty(const std::string& sProperty)
        {
            return m_Encoder->GetProperty(sProperty);
        }
        */


protected:
    hal::Uri                                m_URI;
    std::shared_ptr<EncoderDriverInterface> m_Encoder;

};

} /* namespace */
