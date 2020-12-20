#include "tof_cam635_distance_image.h"

namespace ComLib
{

TofCam635Image::TofCam635ImageType_e TofCam635DistanceImage::getType()
{
  return TofCam635Image::TofCam635ImageType_e::TOFCAM635IMAGE_DISTANCE;
}

unsigned int TofCam635DistanceImage::getDistanceOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  //Make a pointer to the distance
  uint16_t *distance =  (uint16_t *)(&data.data()[dataOffset]);

  return (distance[index] & CommunicationConstants::PixelTofCam635::MASK_OUT_CONFIDENCE);
}

}
