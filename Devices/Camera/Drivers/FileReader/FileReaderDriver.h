#ifndef _FILE_READER_H_
#define _FILE_READER_H_

#include <queue>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "RPG/Devices/Camera/CameraDriverInterface.h"

using namespace std;

class FileReaderDriver : public CameraDriver
{
    public:
        FileReaderDriver();
        virtual ~FileReaderDriver();
        bool Capture( std::vector<rpg::ImageWrapper>& vImages );
        bool Init();

    private:
        static void _ThreadCaptureFunc( FileReaderDriver* pFR );
        //void _Read( std::vector<rpg::ImageWrapper>& vImages );
        bool _Read();
        cv::Mat _OpenPDM( const std::string& FileName );

    private:
        boost::thread*									m_CaptureThread;

        // vector of lists of files
        boost::mutex                                    m_Mutex;
        boost::condition_variable                       m_cBufferEmpty;
        boost::condition_variable                       m_cBufferFull;

        std::queue< std::vector<rpg::ImageWrapper> >    m_qImageBuffer;
        std::vector< std::vector< std::string > >		m_vFileList;
        unsigned int                                    m_nCurrentImageIndex;
        unsigned int                                    m_nStartFrame;
        bool                                            m_bLoop;
        unsigned int                                    m_nNumImages;
        unsigned int                                    m_nNumChannels;
        unsigned int                                    m_nBufferSize;

        //std::vector< bool >                             m_vBufferFree;
};

#endif
