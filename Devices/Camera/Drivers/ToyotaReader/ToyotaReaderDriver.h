#ifndef _TOYOTA_READER_H_
#define _TOYOTA_READER_H_

#include <queue>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <Mvlpp/Mvl.h>

#include "RPG/Devices/Camera/CameraDriverInterface.h"

using namespace std;

class ToyotaReaderDriver : public CameraDriver
{
    enum ColorFormat {RGB, BGR, BAYER, /* BAYER == BAYER_GB */
                BAYER_BG, BAYER_GB, BAYER_RG, BAYER_GR
    };


    struct CameraInfo {
        int                 w;              // width in cells
        int                 h;              // height in cells
        int                 fps;            // estimated fps of hardware
        int                 format;         // BGR / Bayer
        int                 fsize;          // frame size in bytes
        string              sformat;
        string              name;
        mvl::CameraModel*   pCMod;
    };

    public:
        ToyotaReaderDriver();
        virtual ~ToyotaReaderDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        bool Init();

    private:
        static void _ThreadCaptureFunc( ToyotaReaderDriver* pTR );
        bool _Read();
        void _PrintCamInfo();
        int   _GetImageFormat(string& format);
        void _bayer8_to_grey8_half(unsigned char* src, unsigned char* dst, unsigned int srcWidth, unsigned int srcHeight );

    private:
        boost::thread*                                  m_CaptureThread;

        // vector of lists of files
        boost::mutex                                    m_Mutex;
        boost::condition_variable                       m_cBufferEmpty;
        boost::condition_variable                       m_cBufferFull;

        std::queue< std::vector<rpg::ImageWrapper> >    m_qImageBuffer;

        bool                                            m_bLoop;
        bool                                            m_bOutputRectified;
        unsigned int                                    m_uCurrentImageIndex;
        unsigned int                                    m_uStartFrame;
        unsigned int                                    m_uNumImages;
        unsigned int                                    m_uNumChannels;
        unsigned int                                    m_uBufferSize;
        int                                             m_nCvImageReadFlags;

        mvl::StereoRectification                        m_Rectify;
        std::vector<CameraInfo>                         m_vCamerasInfo;
        std::vector<ifstream*>                          m_vChannels;

};

#endif
