#ifndef _FILE_READER_H_
#define _FILE_READER_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"

class FileReaderDriver : public CameraDriver
{
    public:
        FileReaderDriver();
        virtual ~FileReaderDriver();
        bool Capture( std::vector<cv::Mat>& vImages );
        bool Init();
    private:
        // vector of lists of files
        std::vector< std::vector< std::string > >  m_vFileList;
        unsigned int                               m_nCurrentImageIndex;
        unsigned int                               m_nStartFrame;
		unsigned int                               m_nNumImages;
        unsigned int                               m_nNumChannels;
};

#endif
