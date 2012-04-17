#ifndef RPG_IMAGEWRAPPER_H
#define	RPG_IMAGEWRAPPER_H

#include <opencv/cv.h>
#include <RPG/Utils/PropertyMap.h>

namespace rpg {

    struct ImageWrapper {
        cv::Mat         Image;
        PropertyMap     Map;
    };

}

#endif	/* _IMAGEWRAPPER_H_ */

