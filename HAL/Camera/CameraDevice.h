#pragma once

#include <HAL/Messages/ImageArray.h>
#include <HAL/Messages/Image.h>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Devices/DriverFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal 
{

  typedef DriverFactory<CameraDriverInterface> CameraFactory_t;
  typedef DeviceDriverRegistry<CameraDriverInterface> CameraRegistry_t;

  ///////////////////////////////////////////////////////////////////////////////
  // Simplified access to the camera registry 
  CameraRegistry_t& CameraRegistry()
  {
    return hal::DeviceDriverRegistry<CameraDriverInterface>::Registry();
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Simplified access to the camera factory
  CameraFactory_t& CameraFactory( const Uri& uri)
  {
    return CameraRegistry().GetFactory( uri );
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Print known camera drivers
  void ListCameraDrivers()
  {
    CameraRegistry().ListDrivers();
  }

  ///////////////////////////////////////////////////////////////////////////////
  // Print known camera drivers
  bool ListCameraDriverInfo( const std::string& driver )
  {
    CameraFactory_t& factory = CameraFactory(driver);
    if( !&factory ){
      std::cout << "Unknown driver: '" << driver << "'\n";
      return false;
    }
    std::cout <<  "\nHAL '" << driver << "' driver has the following properties:\n";
    factory.PrintDefaultParams();
    std::cout << "\n\n";
    return true;
  }


  ///////////////////////////////////////////////////////////////////////////////
  // Generic camera device
  class Camera : public CameraDriverInterface
  {
    public:
      ///////////////////////////////////////////////////////////////
      Camera()
      {
      }

      Camera( const std::string& uri )
      {
        Init( uri );
      }

      Camera( const hal::Uri& uri )
      {
        Init( uri );
      }

      ///////////////////////////////////////////////////////////////
      ~Camera()
      {
        Clear();
      }

      // all drivers are required to have an init function.
      // In this case (since we're the "device" not a driver we just stub out a
      // dummy implementation.
//      bool Init( const PropertyMap&, const Uri&) { return true; }

      ///////////////////////////////////////////////////////////////
      bool Init( const std::string& uri )
      {
        return Init( hal::Uri(uri) );
      }

      ///////////////////////////////////////////////////////////////
      bool Init( const hal::Uri& uri )
      {
        m_uri = uri;
        CameraFactory_t& factory = CameraFactory( m_uri );
        if( !&factory ){
          std::cerr << "Unknown driver: '" << m_uri.url << "'\n";
          return false;
        }
        m_cam = factory.CreateDriver( device_properties_, m_uri );
        return true;
      }

      ///////////////////////////////////////////////////////////////
      void PrintPropertyMap()
      {
        device_properties_.PrintPropertyMap();
      }


      ///////////////////////////////////////////////////////////////
      void PrintInfo() 
      {
        if( !&m_cam ){
           std::cerr << "Error: CameraDevice has not been initialized\n";
           return;
        }
        m_cam->PrintInfo();
      }

      ///////////////////////////////////////////////////////////////
      inline void Reset()
      {
        Clear();
      }

      ///////////////////////////////////////////////////////////////
      inline void Clear()
      {
        m_cam = nullptr;
      }

      ///////////////////////////////////////////////////////////////
      inline bool Empty() const
      {
        return m_cam == nullptr;
      }

      ///////////////////////////////////////////////////////////////
      std::shared_ptr<CameraDriverInterface> GetInputDriver()
      {
        if( m_cam == nullptr ){
          return nullptr;
        }
        return m_cam->GetInputDriver();
      }

      ///////////////////////////////////////////////////////////////
      bool Capture( hal::CameraMsg& Images )
      {
        if( m_cam == nullptr ){
          return false;
        }
        Images.Clear();
        return m_cam->Capture(Images);
      }

      ///////////////////////////////////////////////////////////////
      bool Capture( hal::ImageArray& Images )
      {
        if( m_cam == nullptr ){
          return false;
        }
        return Capture( Images.Ref() );
      }

      ///////////////////////////////////////////////////////////////
      bool Capture(
          std::vector<cv::Mat>& vImages
          )
      {
        if( m_cam == nullptr ){
          return false;
        }
        std::vector<hal::ImageInfoMsg> vImageInfo;
        return Capture( vImages, vImageInfo );
      }

      ///////////////////////////////////////////////////////////////
      bool Capture(
          std::vector<cv::Mat>& vImages,
          std::vector<hal::ImageInfoMsg>& vImageInfo
          )
      {
        if( m_cam == nullptr ){
          return false;
        }
        static std::shared_ptr<hal::ImageArray> pbImages =
          hal::ImageArray::Create();
        bool bRes = Capture( pbImages->Ref() );
        vImages.resize( pbImages->Size() );
        vImageInfo.resize( pbImages->Size() );
        if( bRes ){
          for (int ii = 0; ii < pbImages->Size(); ++ii) {
            std::shared_ptr<hal::Image> img = pbImages->at(ii);
            vImages[ii] = *img;
            if( img->HasInfo() ){
              vImageInfo[ii] = img->GetInfo();
            }
          }
        }
        return bRes;
      }

      ///////////////////////////////////////////////////////////////
      size_t NumChannels() const
      {
        if( m_cam == nullptr ){
          return 0;
        }
        return m_cam->NumChannels();
      }

      ///////////////////////////////////////////////////////////////
      size_t Width( size_t idx = 0 ) const
      {
        if( m_cam == nullptr ){
          return 0;
        }
        return m_cam->Width(idx);
      }

      ///////////////////////////////////////////////////////////////
      size_t Height( size_t idx = 0 ) const
      {
        if( m_cam == nullptr ){
          return 0;
        }
        return m_cam->Height(idx);
      }

      ///////////////////////////////////////////////////////////////
			hal::Type   PixelType() 
			{
				return  m_cam == nullptr ? (hal::Type)0 :m_cam->PixelType();
			}

			///////////////////////////////////////////////////////////////
			hal::Format PixelFormat() 
			{
				return  m_cam == nullptr ? (hal::Format)0 :m_cam->PixelFormat();
			}

      ///////////////////////////////////////////////////////////////
      // property map accessors
      template<typename T>
        void SetProperty(
            const std::string& key, 
            T value,
            const std::string& description = ""
            )
        {
          device_properties_.SetProperty(key,value,description);
        }


      ///////////////////////////////////////////////////////////////
      template<typename T>
        T GetProperty(const std::string& key, T default_val = T() ) const
        {
          return device_properties_.GetProperty(key,default_val);
        }

      ///////////////////////////////////////////////////////////////
      /// Return raw camera driver pointer, optionally dynamic_cast'd
      /// to another type. Returns NULL if wrong type.
      /// e.g. RectifyDriver* rectify = camera.GetDriver<RectifyDriver>();
      ///      if(rectify) {...}
      template<typename CameraDriverType = CameraDriverInterface>
        CameraDriverType* GetDriver()
        {
          if( m_cam == nullptr ){
            return nullptr;
          }
          CameraDriverInterface* di = m_cam.get();
          return dynamic_cast<CameraDriverType*>(di);
        }

    protected:
      PropertyMap                             device_properties_;
      hal::Uri                                m_uri;
      std::shared_ptr<CameraDriverInterface>  m_cam;
  };

} 
