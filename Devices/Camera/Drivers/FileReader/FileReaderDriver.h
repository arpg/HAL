#ifndef _FILE_READER_H_
#define _FILE_READER_H_

#include <boost/thread.hpp>

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
        void _Read( std::vector<rpg::ImageWrapper>& vImages );
		cv::Mat _OpenPDM( const std::string& FileName );

    private:
		boost::thread*							m_CaptureThread;

		// vector of lists of files
        volatile double                         m_dBufferFilled;

        vector< vector<rpg::ImageWrapper> >     m_vImageBuffer;     
        vector< vector< string > >              m_vFileList;
        unsigned int                            m_nCurrentImageIndex;
        unsigned int                            m_nStartFrame;
		unsigned int                            m_nNumImages;
        unsigned int                            m_nNumChannels;
        unsigned int                            m_nBufferSize;
        volatile unsigned int                   m_nNextRead;
        volatile unsigned int                   m_nNextCapture;
        vector<bool>                            m_vBufferFree;    
};

#endif
