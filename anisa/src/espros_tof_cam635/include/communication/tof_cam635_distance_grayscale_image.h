#ifndef TOFCAM635_DISTANCE_GRAYSCALE_IMAGE_H
#define TOFCAM635_DISTANCE_GRAYSCALE_IMAGE_H

#include "tof_cam635_image.h"

namespace ComLib
{

class TofCam635DistanceGrayscaleImage: public TofCam635Image
{
  public:
    using TofCam635Image::TofCam635Image;
    TofCam635ImageType_e getType();
    unsigned int getDistanceOfPixel(const unsigned int index) const;
    unsigned int getGrayscaleOfPixel(const unsigned int index) const;
};

}

#endif // TOFCAM635_DISTANCE_GRAYSCALE_IMAGE_H
