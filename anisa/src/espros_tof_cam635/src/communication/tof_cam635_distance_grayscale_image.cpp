#include "tof_cam635_distance_grayscale_image.h"

namespace ComLib
{

#pragma pack(push,1)
typedef struct
{
    uint8_t grayscale;
    uint16_t distance;
}distanceGrayscale_t;
#pragma pack(pop)

TofCam635Image::TofCam635ImageType_e TofCam635DistanceGrayscaleImage::getType()
{
  return TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_DISTANCE_GRAYSCALE;
}

unsigned int TofCam635DistanceGrayscaleImage::getDistanceOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  distanceGrayscale_t *distanceGrayscale =  (distanceGrayscale_t *)(&data.data()[dataOffset]);

  return distanceGrayscale[index].distance & CommunicationConstants::PixelTofCam635::MASK_OUT_CONFIDENCE;
}

unsigned int TofCam635DistanceGrayscaleImage::getGrayscaleOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  distanceGrayscale_t *distanceGrayscale =  (distanceGrayscale_t *)(&data.data()[dataOffset]);

  return distanceGrayscale[index].grayscale;
}

}
