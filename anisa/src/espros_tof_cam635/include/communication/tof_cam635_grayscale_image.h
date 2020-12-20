#ifndef TOFCAM635_GRAYSCALE_IMAGE_H
#define TOFCAM635_GRAYSCALE_IMAGE_H

#include "tof_cam635_image.h"

namespace ComLib
{

class TofCam635GrayscaleImage: public TofCam635Image
{
  public:
    using TofCam635Image::TofCam635Image;
    using TofCam635Image::getGrayscaleOfPixel;
    TofCam635ImageType_e getType();    
};

}

#endif // TOFCAM635_GRAYSCALE_IMAGE_H
