#ifndef IMAGE_H
#define IMAGE_H

#include <fstream>

#include <Eigen/Eigen>
#include <PbMsgs/Messages.pb.h>

#define HAVE_OPENCV

#ifdef HAVE_OPENCV
#pragma GCC system_header
#include <opencv.hpp>
#endif

namespace pb {


#ifdef HAVE_OPENCV

void ReadCvMat( const cv::Mat& cvImage, pb::ImageMsg* pbImage )
{
    pbImage->set_data( (const char*)cvImage.data );
    pbImage->set_height( cvImage.rows );
    pbImage->set_width( cvImage.cols );

    if( cvImage.elemSize1() == 1 ) {
        pbImage->set_type( pb::PB_UNSIGNED_BYTE );
    }
    if( cvImage.elemSize1() == 2 ) {
        pbImage->set_type( pb::PB_UNSIGNED_SHORT );
    }
    if( cvImage.elemSize1() == 4 ) {
        pbImage->set_type( pb::PB_FLOAT );
    }

    if( cvImage.channels() == 1 ) {
        pbImage->set_format( pb::PB_LUMINANCE );
    }
    if( cvImage.channels() == 3 ) {
        pbImage->set_format( pb::PB_RGB );
    }
}

cv::Mat WriteCvMat( pb::ImageMsg* pbImage )
{
    int nCvType = 0;
    if( pbImage->type() == pb::PB_BYTE || pbImage->type() == pb::PB_UNSIGNED_BYTE ) {
        if(pbImage->format() == pb::PB_LUMINANCE ) {
            nCvType = CV_8UC1;
        }
        if(pbImage->format() == pb::PB_RGB ) {
            nCvType = CV_8UC3;
        }
    }
    if( pbImage->type() == pb::PB_UNSIGNED_SHORT || pbImage->type() == pb::PB_SHORT ) {
        if(pbImage->format() == pb::PB_LUMINANCE ) {
            nCvType = CV_16UC1;
        }
        if(pbImage->format() == pb::PB_RGB ) {
            nCvType = CV_16UC3;
        }
    }
    if( pbImage->type() == pb::PB_FLOAT ) {
        if(pbImage->format() == pb::PB_LUMINANCE ) {
            nCvType = CV_32FC1;
        }
        if(pbImage->format() == pb::PB_RGB ) {
            nCvType = CV_32FC3;
        }
    }

    return cv::Mat( pbImage->height(), pbImage->width(), nCvType, (void*)pbImage->mutable_data()->data() );
}

void ReadFile( const std::string sFileName, pb::ImageMsg* pbImage )
{
    cv::Mat Image;

    std::string sExtension = sFileName.substr( sFileName.rfind( "." ) + 1 );

    // check if it is our own "portable depth map" format
    if( sExtension == "pdm" ) {

        // magic number P7, portable depthmap, binary
        std::ifstream File( sFileName.c_str() );

        unsigned int        nImgWidth;
        unsigned int        nImgHeight;
        long unsigned int   nImgSize;

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

            Image.create( nImgHeight, nImgWidth, CV_32FC1 );

            File.seekg( File.tellg() + (std::ifstream::pos_type)1, std::ios::beg );
            File.read( (char*)Image.data, nImgSize );
            File.close();
        }
    } else {
        // ... otherwise let OpenCV open it
        Image = cv::imread( sFileName, cv::IMREAD_UNCHANGED );
    }
    ReadCvMat( Image, pbImage );
}

#endif



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Image
{
public:
    Image()
        : m_pData(nullptr), m_nPitch(0), m_pImage(nullptr)
    {
    }

    Image(ImageMsg* Ptr)
        : m_pImage(Ptr)
    {
        m_nPitch = Ptr->pitch() == 0 ? Ptr->width() : Ptr->pitch();
        m_pData = (unsigned char*)( &Ptr->mutable_data()->front() ) + Ptr->offset();
    }

    unsigned int Width() const
    {
        return m_pImage->width();
    }

    unsigned int Height() const
    {
        return m_pImage->height();
    }

    unsigned int Pitch() const
    {
        return m_nPitch;
    }

    int Type() const
    {
        return m_pImage->type();
    }

    int Format() const
    {
        return m_pImage->format();
    }

    double Timestamp() const
    {
        return m_pImage->timestamp();
    }

    const pb::ImageInfoMsg& GetInfo() const
    {
        m_pImage->info();
    }

    unsigned char* data()
    {
        return m_pData;
    }

    unsigned char* RowPtr( unsigned int Idx = 0 ) const
    {
        return m_pData + (Idx * m_nPitch);
    }


#ifdef HAVE_OPENCV
    operator cv::Mat()
    {
        return WriteCvMat(m_pImage);
    }
#endif

    template< typename T >
    T at( unsigned int row, unsigned int col ) const
    {
        return *(T*)( RowPtr(row) + ( col*sizeof(T) ) );
    }

    template< typename T >
    T& at( unsigned int row, unsigned int col )
    {
        return *(T*)( RowPtr(row) + ( col*sizeof(T) ) );
    }

    unsigned char operator()( unsigned int row, unsigned int col  ) const
    {
        return *(RowPtr(row) + col);
    }

    unsigned char& operator()( unsigned int row, unsigned int col  )
    {
        return *(RowPtr(row) + col);
    }
    
protected:
    unsigned char*  m_pData;
    unsigned int    m_nPitch;
    ImageMsg*       m_pImage;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ImageArray
{
public:
    ImageArray() {}

    CameraMsg& ref()
    {
        return m_Message;
    }

    unsigned int Size()
    {
        return m_Message.image_size();
    }

    Image operator[]( unsigned int idx  )
    {
        if( idx < Size() ) {
            return Image(m_Message.mutable_image(idx));
        }

        // TODO: define ensure macro or throw exception
        std::cerr << "error: Image index out of bounds." << std::endl;
        exit(1);
    }

private:
    CameraMsg                       m_Message;
};



}


#endif // IMAGE_H
