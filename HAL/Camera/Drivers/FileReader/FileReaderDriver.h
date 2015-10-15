#pragma once

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <HAL/Camera/CameraDriverInterface.h>
#include <HAL/Utils/Uri.h>

namespace hal {

  class FileReaderDriver : public CameraDriverInterface {
    public:
      FileReaderDriver( const Uri& uri );

			FileReaderDriver( PropertyMap& device_properties );
      ~FileReaderDriver();
      bool Capture( hal::CameraMsg& vImages );
      std::shared_ptr<CameraDriverInterface> GetInputDriver() {
        return std::shared_ptr<CameraDriverInterface>();
      }

			void PrintInfo()
			{
			}

			hal::Type   PixelType()
			{
				return video_type_;
			}

			hal::Format PixelFormat()
			{
				return video_format_;
			}

      size_t NumChannels() const;
      size_t Width( size_t idx = 0 ) const;
      size_t Height( size_t idx = 0 ) const;

    private:
      static void _ThreadCaptureFunc( FileReaderDriver* pFR );
      bool _Read();
      double _GetNextTime();
      double _GetTimestamp(const std::string& sFileName) const;

    private:
      hal::Type                                       video_type_;
      hal::Format                                     video_format_;

			volatile bool                                   m_bShouldRun;
      std::shared_ptr<std::thread>                    m_CaptureThread;

      // vector of lists of files
      std::mutex                                      m_Mutex;
      std::condition_variable                         m_cBufferEmpty;
      std::condition_variable                         m_cBufferFull;

      // TODO refactor using circular buffer
      std::vector< hal::CameraMsg >                   m_vBuffer;
      unsigned int                                    m_nHead;
      unsigned int                                    m_nTail;

      std::queue< hal::CameraMsg >                    m_qImageBuffer;
      std::vector< std::vector< std::string > >       m_vFileList;
      std::string                                     m_sBaseDir;
      unsigned int                                    m_nNumChannels;
      unsigned int                                    m_nCurrentImageIndex;
      bool                                            m_bLoop;
      unsigned int                                    m_nNumImages;
      unsigned int                                    m_nBufferSize;
      int                                             m_iCvImageReadFlags;
      std::string                                     m_sTimeKeeper;
      std::string                                     m_sName;
      std::string                                     m_sId;
      unsigned int                                    m_nFramesProcessed;
      double                                          frequency_;
      std::vector<std::string>                        regexs_;

     // This refers to the container (not parent) CameraDevice's property map 
      PropertyMap&                        device_properties_;
  };
}  // end namespace hal

