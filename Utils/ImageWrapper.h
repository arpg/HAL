#ifndef RPG_IMAGEWRAPPER_H
#define	RPG_IMAGEWRAPPER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <RPG/Utils/PropertyMap.h>

namespace rpg {
    class ImageWrapper;
    ////////////////////////////////////////////////////////////////////////////
    inline ImageWrapper imread( const std::string& sImageFileName, 
                                const std::string& sExtraInfoFileName,
                                int nFlags = 1 //<Input: same flag convention as in OpenCV (>0 colour, 0 greyscale, <0 as is)
                                );

    ////////////////////////////////////////////////////////////////////////////
    inline ImageWrapper imread( const std::string& sImageFileName, 
                                bool bReadExtraInfo = true,
                                int nFlags = 1 //<Input: same flag convention as in OpenCV (>0 colour, 0 greyscale, <0 as is)
                                );

    ////////////////////////////////////////////////////////////////////////////
    class ImageWrapper {
    public:
        cv::Mat         Image;
        PropertyMap     Map;

        /// Check if the image has any data
        bool empty() { return Image.data == NULL; }

        int width() { return Image.cols; }
        int height() { return Image.rows; }
        int widthStep() { return static_cast<int>( Image.step ); }

        struct ImageWrapper clone() { struct ImageWrapper ret = *this; this->Image = ret.Image.clone(); return *this; }

        /// Write an image to sImageName and the property map to sExtraInfoName.
        /// No checks are made for overwriting.
        inline bool write( const std::string& sImageName, const std::string& sExtraInfoName );

        /// If bWriteExtraInfo is set to true, this call will automatically deduce the 
        /// sExtraInfoName from the sImageName by replacing the
        /// extension by ".txt" (if not extension is found, ".txt" will be
        /// appended). No checks are made for overwriting.
        inline bool write( const std::string& sImageName, bool bWriteExtraInfo = true );

        /// Read an image from sImageName and the property map from sExtraInfoName.
        /// nFlags can be used to automatically load in color or image (or keep as such - default).
        inline void read( const std::string& sImageFileName, 
                          const std::string& sExtraInfoFileName,
                          int nFlags = 1 ) {
            *this = imread( sImageFileName, sExtraInfoFileName, nFlags );
        }

        /// Read an image from sImageName and the property map from sExtraInfoName.
        /// If bReadExtraInfo is set to true, this call will automatically deduce the 
        /// sExtraInfoName from the sImageName by replacing the
        /// extension by ".txt" (if not extension is found, ".txt" will be
        /// appended).
        /// nFlags can be used to automatically load in color or image (or keep as such - default).
        inline void read( const std::string& sImageFileName, 
                          bool bReadExtraInfo = true,
                          int nFlags = 1 //<Input: same flag convention as in OpenCV (>0 colour, 0 greyscale, <0 as is)
                          ) {
            *this = imread( sImageFileName, bReadExtraInfo, nFlags );
        }
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

////////////////////////////////////////////////////////////////////////////////
inline rpg::ImageWrapper rpg::imread( const std::string& sImageFileName, 
                                      const std::string& sExtraInfoFileName,
                                      int nFlags
                                      ) {
    ImageWrapper retImage;
    retImage.Image = cv::imread( sImageFileName.c_str(), nFlags );
    if( sExtraInfoFileName != "" ) {
        cv::FileStorage oFile( sExtraInfoFileName.c_str(), cv::FileStorage::READ );

        cv::FileNode r = oFile.root();
        cv::FileNodeIterator it_r_b = r.begin();
        cv::FileNodeIterator it_r_e = r.end();

        for( ; it_r_b != it_r_e; it_r_b++ ) {
            cv::FileNode fnode = *it_r_b;
            for( cv::FileNodeIterator it = fnode.begin(); it != fnode.end(); it++ ) {
                //std::cout << (*it).name() << std::endl;
                //std::cout << (std::string)oFile[ (*it).name() ] << std::endl;
                retImage.Map.SetProperty( (*it).name(), 
                                          (std::string)oFile[ (*it).name() ] );
            }
        }
    }
    return retImage;
}

////////////////////////////////////////////////////////////////////////////////
inline rpg::ImageWrapper rpg::imread
( const std::string& sImageFileName, bool bReadExtraInfo, int nFlags
  ) {
    std::string sExtraInfoName = "";
    if( bReadExtraInfo ) {
        sExtraInfoName = sImageFileName;
        sExtraInfoName.erase( sExtraInfoName.rfind( '.' ) );
        sExtraInfoName += ".txt";
    }
    return imread( sImageFileName, sExtraInfoName, nFlags );
}

#endif	/* RPG_IMAGEWRAPPER_H */


