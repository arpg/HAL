#ifndef _FILE_READER_H_
#define _FILE_READER_H_

#include <vector>
#include <queue>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "HAL/Camera/CameraDriverInterface.h"

namespace hal {

class FileReaderDriver : public CameraDriverInterface
{
    public:
        FileReaderDriver(const std::vector<std::string>& ChannelRegex, size_t StartFrame = 0, bool Loop = false, size_t BufferSize = 35, int cvFlags = 0 /*cv::IMREAD_UNCHANGED*/);
        ~FileReaderDriver();
        bool Capture( pb::CameraMsg& vImages );

    private:
        static void _ThreadCaptureFunc( FileReaderDriver* pFR );
        bool _Read();
        double _GetNextTime();

    private:
        boost::thread*									m_CaptureThread;

        // vector of lists of files
        boost::mutex                                      m_Mutex;
        boost::condition_variable                         m_cBufferEmpty;
        boost::condition_variable                         m_cBufferFull;

        std::vector< pb::CameraMsg >                    m_vBuffer;
        unsigned int                                    m_nHead;
        unsigned int                                    m_nTail;

        std::queue< pb::CameraMsg >                     m_qImageBuffer;
        std::vector< std::vector< std::string > >		m_vFileList;
        unsigned int                                    m_nNumChannels;
        unsigned int                                    m_nStartFrame;
        unsigned int                                    m_nCurrentImageIndex;
        bool                                            m_bLoop;
        unsigned int                                    m_nNumImages;
        unsigned int                                    m_nBufferSize;
        int                                             m_iCvImageReadFlags;
        std::string                                     m_sTimeKeeper;
};

}

#endif
