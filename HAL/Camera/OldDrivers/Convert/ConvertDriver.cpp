#include "ConvertDriver.h"
#include "HAL/Devices/DeviceException.h"

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace hal
{

ConvertDriver::ConvertDriver(
    std::shared_ptr<CameraDriverInterface> Input,
    const Uri& uri
//    const PropertyMap& params 
    )
  : m_Input(Input),
    m_nOutCvType(-1),
    m_nNumChannels(Input->NumChannels())
{
  // sat sane default parameters
  CameraDriverInterface::SetDefaultProperties({
      {"fmt", "MONO8", "Output video format: MONO8, RGB8, BGR8"},
      {"range", "1", "Range of values of 16 and 32 bit images: ir (1023), "
      "depth (4500) or numerical value"},
      {"size", "0x0", "Capture resolution (0x0 for unused)."},
      {"channel", "-1", "Particular channel to convert (-1 for all)."} 
      });
  if( !CameraDriverInterface::ParseUriProperties( uri.properties ) ){
    std::cerr << "ConvertDriver knows about the following properties:\n";
    CameraDriverInterface::PrintPropertyMap();
    return;
  }

  // pluck properties back out
  m_sFormat = CameraDriverInterface::GetProperty<std::string>("fmt", "MONO8");
  m_Dims = CameraDriverInterface::GetProperty<ImageDim>("size", ImageDim(0, 0));
  m_iChannel = CameraDriverInterface::GetProperty<int>("channel", -1);

  // set rage based on a flag
  std::string sRange = CameraDriverInterface::GetProperty<std::string>("range", "1");
  if(sRange == "ir")
    m_dRange = 1023; // OpenNi uses the 10 l.s.bits only (range [0, 1023])
  else if(sRange == "ir2")
    m_dRange = 20000; // libfreenect2 uses this value
  else if(sRange == "depth")
    m_dRange = 4500; // max range (mm) of asus xtion pro live
  else {
    m_dRange = strtod(sRange.c_str(), nullptr);
    if(m_dRange == 0.) m_dRange = 1.;
  }




  // Set the correct image size on the output interface, considering the request
  // to resize the images
  for(size_t i = 0; i < Input->NumChannels(); ++i) {
    m_nImgWidth.push_back(Input->Width(i));
    m_nImgHeight.push_back(Input->Height(i));
    m_nOrigImgWidth.push_back(Input->Width(i));
    m_nOrigImgHeight.push_back(Input->Height(i));

    if (m_iChannel != -1) {
      if (m_iChannel != (int)i) {
        continue;
      }
    }

    const bool resize_requested = (m_Dims.x != 0 || m_Dims.y != 0);
    if (resize_requested) {
      m_nImgWidth[i] = m_Dims.x;
      m_nImgHeight[i] = m_Dims.y;
    }
  }

  // Guess output color coding
  if( m_sFormat == "MONO8" ) {
    m_nOutCvType = CV_8UC1;
    m_nOutPbType = hal::Format::PB_LUMINANCE;
  } else if( m_sFormat == "RGB8" ) {
    m_nOutCvType = CV_8UC3;
    m_nOutPbType = hal::Format::PB_RGB;
  } else if( m_sFormat == "BGR8" ) {
    m_nOutCvType = CV_8UC3;
    m_nOutPbType = hal::Format::PB_BGR;
  }

  if( m_nOutCvType == -1 ){
    // throw DeviceException("HAL: Error! Unknown target format: " + m_sFormat);
    std::cerr << "HAL: Error! Unknown target format: " + m_sFormat;
  }
}

bool ConvertDriver::Capture( hal::CameraMsg& vImages )
{
  m_Message.Clear();
  bool srcGood = m_Input->Capture( m_Message );

  if (!srcGood)
    return false;

  // Guess source color coding.
  if( m_nCvType.empty() ) {
    for(int i = 0; i < m_Message.image_size(); ++i) {
      int cvtype = -1;
      hal::Format pbtype = m_Message.image(i).format();
      int channels = 0;

      if( m_Message.image(i).format() == hal::PB_LUMINANCE )
        channels = 1;
      else if( m_Message.image(i).format() == hal::PB_RGB ||
          m_Message.image(i).format() == hal::PB_BGR )
        channels = 3;

      if( channels != 0 ) {
        if( m_Message.image(i).type() == hal::PB_BYTE ||
            m_Message.image(i).type() == hal::PB_UNSIGNED_BYTE )
          cvtype = (channels == 1 ? CV_8UC1 : CV_8UC3);
        else if( m_Message.image(i).type() == hal::PB_UNSIGNED_SHORT ||
            m_Message.image(i).type() == hal::PB_SHORT )
          cvtype = (channels == 1 ? CV_16UC1 : CV_16UC3);
        else if( m_Message.image(i).type() == hal::PB_FLOAT )
          cvtype = (channels == 1 ? CV_32FC1 : CV_32FC3);
      }

      m_nCvType.push_back(cvtype);
      m_nPbType.push_back(pbtype);

      if( cvtype == -1 ) {
        std::cerr << "HAL: Error! Could not guess source color coding of "
          "channel " << i << ". Is it RAW?" << std::endl;
      }
    }
  }

  // Prepare return images.
  vImages.set_device_time(m_Message.device_time());
  vImages.set_system_time(m_Message.system_time());

  for(size_t ii = 0; ii < m_nNumChannels; ++ii) {
    hal::ImageMsg* pbImg = vImages.add_image();

    // If the user has specified to convert a single channel only,
    // gate it here
    if (m_iChannel != -1) {
      if (m_iChannel != (int)ii) {
        *pbImg = m_Message.image(ii);
        continue;
      }
    }

    if( m_nCvType[ii] == -1 ) { // this image cannot be converted
      *pbImg = m_Message.image(ii);
      continue;
    }

    const bool resize_requested = (m_Dims.x != 0 || m_Dims.y != 0);
    size_t final_width, final_height;
    if (resize_requested) {
      final_width = m_Dims.x;
      final_height = m_Dims.y;
    } else {
      final_width = m_nOrigImgWidth[ii];
      final_height = m_nOrigImgHeight[ii];
    }
    pbImg->set_width( final_width );
    pbImg->set_height( final_height );
    pbImg->set_type( hal::PB_UNSIGNED_BYTE );
    pbImg->set_format( m_nOutPbType );
    pbImg->mutable_data()->resize(final_width * final_height *
        (m_nOutCvType == CV_8UC1 ? 1 : 3) );

    pbImg->set_timestamp( m_Message.image(ii).timestamp() );
    pbImg->set_serial_number( m_Message.image(ii).serial_number() );

    cv::Mat s_origImg(m_nOrigImgHeight[ii], m_nOrigImgWidth[ii], m_nCvType[ii],
        (void*)m_Message.mutable_image(ii)->data().data());

    cv::Mat sImg;
    if (resize_requested) {
      cv::resize(s_origImg, sImg, cv::Size(final_width, final_height));
      m_nImgWidth[ii] = final_width;
      m_nImgHeight[ii] = final_height;
    } else {
      sImg = s_origImg;
    }

    cv::Mat dImg(final_height, final_width, m_nOutCvType,
        (void*)pbImg->mutable_data()->data());

    // note: cv::cvtColor cannot convert between depth types and
    // cv::Mat::convertTo cannot change the number of channels
    cv::Mat aux;
    switch( m_nCvType[ii] ) {
      case CV_8UC1:
        if( m_nOutCvType == CV_8UC1 )
          std::copy(sImg.begin<unsigned char>(), sImg.end<unsigned char>(),
              dImg.begin<unsigned char>());
        else
          cv::cvtColor(sImg, dImg,
              (m_nOutPbType == hal::Format::PB_RGB ? CV_GRAY2RGB : CV_GRAY2BGR));
        break;

      case CV_8UC3:
        if( m_nOutCvType == CV_8UC1 )
          cv::cvtColor(sImg, dImg,
              (m_nPbType[ii] == hal::Format::PB_RGB ? CV_RGB2GRAY : CV_BGR2GRAY));
        else {
          if( m_nPbType[ii] == m_nOutPbType )
            std::copy(sImg.begin<unsigned char>(), sImg.end<unsigned char>(),
                dImg.begin<unsigned char>());
          else
            cv::cvtColor(sImg, dImg,
                (m_nPbType[ii] == hal::Format::PB_RGB ? CV_RGB2BGR : CV_BGR2RGB));
        }
        break;

      case CV_16UC1:
        sImg.convertTo(aux, CV_64FC1);
        if( m_nOutCvType == CV_8UC1 )
          aux.convertTo(dImg, CV_8UC1, 255. / m_dRange);
        else {
          aux.convertTo(aux, CV_8UC1, 255. / m_dRange);
          cv::cvtColor(aux, dImg,
              (m_nOutPbType == hal::Format::PB_RGB ? CV_GRAY2RGB : CV_GRAY2BGR));
        }
        break;

      case CV_16UC3:
        sImg.convertTo(aux, CV_64FC3);
        if( m_nOutCvType == CV_8UC1 ) {
          aux.convertTo(aux, CV_8UC3, 255. / m_dRange);
          cv::cvtColor(aux, dImg,
              (m_nPbType[ii] == hal::Format::PB_RGB ? CV_RGB2GRAY : CV_BGR2GRAY));
        } else {
          if( m_nPbType[ii] == m_nOutPbType )
            aux.convertTo(dImg, CV_8UC3, 255. / m_dRange);
          else {
            aux.convertTo(aux, CV_8UC3, 255. / m_dRange);
            cv::cvtColor(aux, dImg,
                (m_nPbType[ii] == hal::Format::PB_RGB ? CV_RGB2BGR : CV_BGR2RGB));
          }
        }
        break;

      case CV_32FC1:
        if( m_nOutCvType == CV_8UC1 ) {
          sImg.convertTo(dImg, CV_8UC1, 255. / m_dRange);
        } else {
          sImg.convertTo(aux, CV_8UC1, 255. / m_dRange);
          cv::cvtColor(aux, dImg,
              (m_nOutPbType == hal::Format::PB_RGB ? CV_GRAY2RGB : CV_GRAY2BGR));
        }
        break;

      case CV_32FC3:
        if( m_nOutCvType == CV_8UC1 ) {
          sImg.convertTo(aux, CV_8UC3, 255. / m_dRange);
          cv::cvtColor(aux, dImg,
              (m_nPbType[ii] == hal::Format::PB_RGB ? CV_RGB2GRAY : CV_BGR2GRAY));
        } else {
          if( m_nPbType[ii] == m_nOutPbType )
            sImg.convertTo(dImg, CV_8UC3, 255. / m_dRange);
          else {
            sImg.convertTo(aux, CV_8UC3, 255. / m_dRange);
            cv::cvtColor(aux, dImg,
                (m_nPbType[ii] == hal::Format::PB_RGB ? CV_RGB2BGR : CV_BGR2RGB));
          }
        }
        break;
    }
  }

  return true;
}

size_t ConvertDriver::NumChannels() const
{
  return m_nNumChannels;
}

size_t ConvertDriver::Width( size_t idx ) const
{
  return m_nImgWidth[idx];
}

size_t ConvertDriver::Height( size_t idx ) const
{
  return m_nImgHeight[idx];
}

} // namespace
