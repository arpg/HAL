#ifndef RPG_IMAGEWRAPPER_H
#define	RPG_IMAGEWRAPPER_H

#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <RPG/Utils/PropertyMap.h>

namespace rpg {

    class ImageWrapper {
    public:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Read an image from sImageFileName. If bReadExtraInfo is set to true, this call will automatically deduce the
        /// file holding the extra info by replacing the extension by ".txt".
        /// nFlags can be used to automatically load in color or image (or keep as such - default).
        inline void read( const std::string&  sImageFileName,         //< Input: File Name
                          bool                bReadExtraInfo = true,  //< Input: If ExtraInfo file should be read or not
                          int                 nFlags = -1             //< Input: OpenCV flags (>0 color, 0 greyscale, <0 as is)
                          );

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Read an image from sImageFileName and the property map from sExtraInfoName.
        /// nFlags can be used to automatically load in color or image (or keep as such - default).
        inline void read( const std::string&  sImageFileName,      //< Input: Image File Name
                          const std::string&  sExtraInfoFileName,  //< Input: Extra Info File Name
                          int                 nFlags = -1          //< Input: OpenCV flags (>0 color, 0 greyscale, <0 as is)
                          );

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// If bWriteExtraInfo is set to true, this call will automatically deduce the sExtraInfoName from the sImageName by
        /// replacing the extension by ".txt" (if no extension is found, ".txt" will be appended).
        /// No checks are made for overwriting.
        inline bool write( const std::string&       sImageFileName,             //< Input: Image File Name
                           bool                     bWriteExtraInfo = true      //< Input: True if ExtraInfo should be written
                           );

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Write an image to sImageName and the property map to sExtraInfoName.
        /// No checks are made for overwriting.
        inline bool write( const std::string&       sImageFileName,             //< Input: Image File Name
                           const std::string&       sExtraInfoFileName          //< Input: Extra Info File Name
                           );




        ////////////////////////////////////////////////////////////////////////////
        // AUXILARY FUNCTIONS
        ////////////////////////////////////////////////////////////////////////////

        /// Return Image's data pointer
        inline unsigned char* data() { return Image.data; }

        /// Check if the image has any data
        inline bool empty() { return Image.data == NULL; }

        /// Return image width
        inline int width() const { return Image.cols; }

        /// Return image height
        inline int height() const { return Image.rows; }

        /// Return image step
        inline int widthStep() { return static_cast<int>( Image.step ); }

        /// Image clone wrapper... missing cloning property map
        inline ImageWrapper clone() {
            ImageWrapper ret;
            ret.Image = Image.clone();
            ret.Map = Map;
            return ret;
        }


    private:

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Open a "Portable Depthmap" image.
        inline cv::Mat _ReadPDM(
                const std::string&                  FileName    //< Input: Image File Name
                );

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// Save a "Portable Depthmap" image.
        inline bool _WritePDM(
                const std::string&                  FileName,   //< Input: Image File Name
                const cv::Mat&                      Image       //< Input: Image
                );

    public:
        cv::Mat             Image;
        PropertyMap         Map;


    }; /* ImageWrapper Class */


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline bool ImageWrapper::write(
            const std::string&          sImageFileName,
            bool                        bWriteExtraInfo
            )
    {
        if( bWriteExtraInfo ) {
            std::string sExtraInfoName = sImageFileName;
            sExtraInfoName.erase( sExtraInfoName.rfind( '.' ) );
            sExtraInfoName += ".txt";
            return write( sImageFileName, sExtraInfoName );
        }
        return write( sImageFileName, "" );
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline bool ImageWrapper::write(
            const std::string&          sImageFileName,
            const std::string&          sExtraInfoFileName
            )
    {
        std::string sExtension = sImageFileName.substr( sImageFileName.rfind( "." ) + 1 );

        bool bImgSuccess;

        // check if it is our own "portable depth map" format
        if( sExtension == "pdm" ) {
             bImgSuccess = _WritePDM( sImageFileName, Image );
        } else {
            // ... otherwise let OpenCV open it
            bImgSuccess = cv::imwrite( sImageFileName.c_str(), Image );
        }

        if( !bImgSuccess ) {
            return false;
        }

        // check if they want us to write the extra info to a file

        if( sExtraInfoFileName != "" ) {
            cv::FileStorage oFile( sExtraInfoFileName.c_str(), cv::FileStorage::WRITE );
            if( !oFile.isOpened() ) {
                return false;
            }

            std::map<std::string,std::string>& ppMap = Map.GetPropertyMap();
            for( std::map<std::string,std::string>::iterator it = ppMap.begin(); it != ppMap.end(); it++ ) {
                oFile << it->first << it->second;
            }
        }
        return true;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void ImageWrapper::read(
            const std::string&              sImageFileName,
            bool                            bReadExtraInfo,
            int                             nFlags
            )
    {
        std::string sExtraInfoName = "";
        if( bReadExtraInfo ) {
            sExtraInfoName = sImageFileName;
            sExtraInfoName.erase( sExtraInfoName.rfind( '.' ) );
            sExtraInfoName += ".txt";
        }
        return read( sImageFileName, sExtraInfoName, nFlags );
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void ImageWrapper::read(
            const std::string&              sImageFileName,
            const std::string&              sExtraInfoFileName,
            int                             nFlags
            )
    {
        std::string sExtension = sImageFileName.substr( sImageFileName.rfind( "." ) + 1 );

        // check if it is our own "portable depth map" format
        if( sExtension == "pdm" ) {
            Image = _ReadPDM( sImageFileName );
        } else {
            // ... otherwise let OpenCV open it
            Image = cv::imread( sImageFileName, nFlags );
        }

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
                    Map.SetProperty( (*it).name(), (std::string)oFile[ (*it).name() ] );
                }
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline cv::Mat ImageWrapper::_ReadPDM(
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

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline bool ImageWrapper::_WritePDM(
            const std::string&              FileName,
            const cv::Mat&                  Image
            )
    {
        if( Image.type() != CV_32FC1 ) {
            return false;
        }

        std::ofstream pFile( FileName.c_str(), std::ios::out | std::ios::binary );
        pFile << "P7" << std::endl;
        pFile << Image.cols << " " << Image.rows << std::endl;
        const unsigned int Size = Image.elemSize1() * Image.rows * Image.cols;
        pFile << 4294967295 << std::endl;
        pFile.write( (const char*)Image.data, Size );
        pFile.close();
        return true;
    }


} /* RPG namespace */

#endif	/* RPG_IMAGEWRAPPER_H */
