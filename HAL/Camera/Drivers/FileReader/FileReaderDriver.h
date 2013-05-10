#ifndef _FILE_READER_H_
#define _FILE_READER_H_

#include <vector>
#include <queue>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "HAL/Camera/CameraDriverInterface.h"

namespace hal {

class FileReaderDriver : public CameraDriver
{
    public:
        FileReaderDriver();
        virtual ~FileReaderDriver();
        bool Capture( pb::CameraMsg& vImages );
        void PrintInfo();
        bool Init();

    private:
        static void _ThreadCaptureFunc( FileReaderDriver* pFR );
        bool _Read();
        inline cv::Mat _ReadFile(
                const std::string&              sImageFileName,
                int                             nFlags
                );
        inline cv::Mat _ReadPDM(
                const std::string&              FileName
                );
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
        unsigned int                                    m_nCurrentImageIndex;
        unsigned int                                    m_nStartFrame;
        bool                                            m_bLoop;
        unsigned int                                    m_nNumImages;
        unsigned int                                    m_nNumChannels;
        unsigned int                                    m_nBufferSize;
        int                                             m_iCvImageReadFlags;
        std::string                                     m_sTimeKeeper;
};

}

#endif
