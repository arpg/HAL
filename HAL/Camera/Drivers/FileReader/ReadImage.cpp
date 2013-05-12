#include "ReadImage.h"

#include <fstream>

namespace hal
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat _ReadFile(
        const std::string&              sImageFileName,
        int                             nFlags
        )
{
    std::string sExtension = sImageFileName.substr( sImageFileName.rfind( "." ) + 1 );

    // check if it is our own "portable depth map" format
    if( sExtension == "pdm" ) {
        return _ReadPDM( sImageFileName );
    } else {
        // ... otherwise let OpenCV open it
        return cv::imread( sImageFileName, nFlags );
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat _ReadPDM(
        const std::string&              FileName
        )
{
    // magic number P7, portable depthmap, binary
    std::ifstream File( FileName.c_str() );

    unsigned int        nImgWidth;
    unsigned int        nImgHeight;
    long unsigned int   nImgSize;

    cv::Mat DepthImg;

    if( File.is_open() ) {
        std::string sType;
        File >> sType;
        File >> nImgWidth;
        File >> nImgHeight;
        File >> nImgSize;

        // the actual PGM/PPM expects this as the next field:
        //		nImgSize++;
        //		nImgSize = (log( nImgSize ) / log(2)) / 8.0;

        // but ours has the actual size (4 bytes of float * pixels):
        nImgSize = 4 * nImgWidth * nImgHeight;

        DepthImg = cv::Mat( nImgHeight, nImgWidth, CV_32FC1 );

        File.seekg( File.tellg() + (std::ifstream::pos_type)1, std::ios::beg );
        File.read( (char*)DepthImg.data, nImgSize );
        File.close();
    }
    return DepthImg;
}

}
