#ifndef RPG_IMAGEWRAPPER_H
#define	RPG_IMAGEWRAPPER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <RPG/Utils/PropertyMap.h>

namespace rpg {

    class ImageWrapper {
    public:
        cv::Mat         Image;
        PropertyMap     Map;

        /// Write an image to sImageName and the property map to sExtraInfoName.
        /// No checks are made for overwriting.
        inline bool write( const std::string& sImageName, const std::string& sExtraInfoName );

        /// If bWriteExtraInfo is set to true, this call will automatically deduce the 
        /// sExtraInfoName from the sImageName by replacing the
        // extension by ".txt" (if not extension is found, ".txt" will be
        // appended). No checks are made for overwriting.
        inline bool write( const std::string& sImageName, bool bWriteExtraInfo = true );
    };

    ////////////////////////////////////////////////////////////////////////////////
    bool ImageWrapper::write( const std::string& sImageName, const std::string& sExtraInfoName ) {
        bool bSuccess = cv::imwrite( sImageName.c_str(), Image );
        if( !bSuccess) { return bSuccess; }
        cv::FileStorage oFile( sExtraInfoName.c_str(), cv::FileStorage::WRITE );
        if( !oFile.isOpened() ) { return false; }
        output( oFile, Map );
        return bSuccess;
    }

    ////////////////////////////////////////////////////////////////////////////////
    bool ImageWrapper::write( const std::string& sImageName, bool bWriteExtraInfo ) {
        if( bWriteExtraInfo ) {
            std::string sExtraInfoName = sImageName;
            sExtraInfoName.erase( sExtraInfoName.rfind( '.' ) );
            sExtraInfoName += ".txt";
            return write( sImageName, sExtraInfoName );
        }
        else {
            return cv::imwrite( sImageName.c_str(), Image );
        }
        return true;
    }
}

#endif	/* RPG_IMAGEWRAPPER_H */


