#pragma once

#include <HAL/IMU/IMUDriverInterface.h>
#include <HAL/Devices/DriverFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal 
{
	typedef DriverFactory<IMUDriverInterface> IMUFactory_t;
	typedef DeviceDriverRegistry<IMUDriverInterface> IMURegistry_t;

	///////////////////////////////////////////////////////////////////////////////
	// Simplified access to the imu registry 
	inline IMURegistry_t& IMURegistry()
	{
		return hal::DeviceDriverRegistry<IMUDriverInterface>::Registry();
	}

	///////////////////////////////////////////////////////////////////////////////
	// Simplified access to the imu factory
	inline IMUFactory_t& IMUFactory( const Uri& uri)
	{
		return IMURegistry().GetFactory( uri );
	}

	// Generic IMU device
	class IMU : public IMUDriverInterface {
		public:
			IMU() {}

			IMU(const std::string& uri) : m_uri(uri) 
  		{
  			Init( uri );
  		}

			bool Init( const std::string& uri )
			{
        m_uri = uri;
        IMUFactory_t& factory = IMUFactory( m_uri );
        if( !&factory ){
          std::cerr << "Unknown driver: '" << m_uri.url << "'\n";
          return false;
        }
        m_IMU = factory.CreateDriver( device_properties_, m_uri );
        return true;
			}

			~IMU() {
				Clear();
			}

      ///////////////////////////////////////////////////////////////
      void PrintInfo() 
      {
        if( !&m_IMU ){
           std::cerr << "Error: IMUDevice has not been initialized\n";
           return;
        }
        m_IMU->PrintInfo();
      }

 
			inline void Reset() {
				Clear();
				// m_IMU = DeviceDriverRegistry<IMUDriverInterface>::Instance().Create(m_uri);
				// RegisterIMUDataCallback(m_callback);
			}

			void Clear() {
				m_IMU = nullptr;
			}

			void RegisterIMUDataCallback(IMUDriverDataCallback callback) 
      {
				m_callback = callback;
				if( m_IMU ){
					m_IMU->RegisterIMUDataCallback( callback );
				}else{
					std::cerr << "ERROR: no driver initialized!\n";
				}
				return;
			}

			bool IsRunning() const override {
				return m_IMU->IsRunning();
			}

		protected:
      PropertyMap                             device_properties_;
			hal::Uri                                m_uri;
			std::shared_ptr<IMUDriverInterface>     m_IMU;
			IMUDriverDataCallback                   m_callback;
	};
} /* namespace hal */
