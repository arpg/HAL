#ifndef IMAGEWRAPPER_H
#define	IMAGEWRAPPEr_H

#include <opencv/cv.h>
#include <RPG/Utils/PropertyMap.h>

namespace rpg {

    struct ImageWrapper {
        cv::Mat         Image;
        PropertyMap     Map;
    };

}

#endif	/* IMAGEWRAPPER_H */

