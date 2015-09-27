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
      std::cout << "Unknown driver: '" << driver << "'";
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

      ///////////////////////////////////////////////////////////////
      Camera(const std::string& uri)
        : m_uri(uri)
      {
        CameraFactory_t& factory = CameraFactory( m_uri );
        m_cam = factory.CreateDriver( m_uri );
      }

      ///////////////////////////////////////////////////////////////
      Camera(const hal::Uri& uri)
        : m_uri(uri)
      {
        CameraFactory_t& factory = CameraFactory( m_uri );
        m_cam = factory.CreateDriver( m_uri );
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
//        m_cam = DeviceRegistry<CameraDriverInterface>::Instance().Create(m_uri);
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
        return m_cam->GetInputDriver();
      }

      ///////////////////////////////////////////////////////////////
      bool Capture( hal::CameraMsg& Images )
      {
        Images.Clear();
        return m_cam->Capture(Images);
      }

      ///////////////////////////////////////////////////////////////
      bool Capture( hal::ImageArray& Images )
      {
        return Capture( Images.Ref() );
      }

      ///////////////////////////////////////////////////////////////
      bool Capture(
          std::vector<cv::Mat>& vImages
          )
      {
        std::vector<hal::ImageInfoMsg> vImageInfo;
        return Capture( vImages, vImageInfo );
      }

      ///////////////////////////////////////////////////////////////
      bool Capture(
          std::vector<cv::Mat>& vImages,
          std::vector<hal::ImageInfoMsg>& vImageInfo
          )
      {
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
        return m_cam->NumChannels();
      }

      ///////////////////////////////////////////////////////////////
      size_t Width( size_t idx = 0 ) const
      {
        return m_cam->Width(idx);
      }

      ///////////////////////////////////////////////////////////////
      size_t Height( size_t idx = 0 ) const
      {
        return m_cam->Height(idx);
      }

      ///////////////////////////////////////////////////////////////
      /// Return raw camera driver pointer, optionally dynamic_cast'd
      /// to another type. Returns NULL if wrong type.
      /// e.g. RectifyDriver* rectify = camera.GetDriver<RectifyDriver>();
      ///      if(rectify) {...}
      template<typename CameraDriverType = CameraDriverInterface>
        CameraDriverType* GetDriver()
        {
          CameraDriverInterface* di = m_cam.get();
          return dynamic_cast<CameraDriverType*>(di);
        }

    protected:
      hal::Uri                                m_uri;
      std::shared_ptr<CameraDriverInterface>  m_cam;
  };

} 
