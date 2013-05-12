#pragma once

#include <string>
#include <opencv2/opencv.hpp>

namespace hal
{

cv::Mat _ReadFile(
        const std::string&              sImageFileName,
        int                             nFlags
        );

cv::Mat _ReadPDM(
        const std::string&              FileName
        );

}
