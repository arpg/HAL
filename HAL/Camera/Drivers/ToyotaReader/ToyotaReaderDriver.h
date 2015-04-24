#ifndef _TOYOTA_READER_H_
#define _TOYOTA_READER_H_

#include <queue>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "HAL/Camera/CameraDriverInterface.h"

namespace hal {

class ToyotaReaderDriver : public CameraDriver
{
    enum ColorFormat {RGB, BGR, BAYER, /* BAYER == BAYER_GB */
                BAYER_BG, BAYER_GB, BAYER_RG, BAYER_GR, GRAY
    };


    struct CameraInfo {
        int                 w;              // width in cells
        int                 h;              // height in cells
        int                 fps;            // estimated fps of hardware
        int                 format;         // BGR / Bayer
        int                 fsize;          // frame size in bytes
        std::string         sformat;
        std::string         name;
        cv::Mat             RectMapRow;
        cv::Mat             RectMapCol;
    };

    public:
        ToyotaReaderDriver();
        virtual ~ToyotaReaderDriver();
        bool Capture( hal::CameraMsg& vImages );
        void PrintInfo();
        bool Init();

    private:
        static void _ThreadCaptureFunc( ToyotaReaderDriver* pTR );
        bool _Read();
        void _PrintCamInfo();
        int   _GetImageFormat( std::string& format );

    private:
        boost::thread*                                  m_CaptureThread;

        // vector of lists of files
        boost::mutex                                    m_Mutex;
        boost::condition_variable                       m_cBufferEmpty;
        boost::condition_variable                       m_cBufferFull;

        std::queue< hal::CameraMsg >                     m_qImageBuffer;

        bool                                            m_bLoop;
        bool                                            m_bOutputRectified;
        bool                                            m_bReadTimestamps;
        unsigned int                                    m_uCurrentImageIndex;
        unsigned int                                    m_uStartFrame;
        unsigned int                                    m_uNumChannels;
        unsigned int                                    m_uBufferSize;

        std::vector< CameraInfo* >                      m_vCamerasInfo;
        std::vector< std::ifstream* >                   m_vChannels;
        std::vector< std::ifstream* >                   m_vTimes;
};

}

#endif
