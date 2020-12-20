#include "tof_cam635_distance_amplitude_image.h"

namespace ComLib
{

TofCam635Image::TofCam635ImageType_e TofCam635DistanceAmplitudeImage::getType()
{
  return TofCam635Image::TofCam635ImageType_e::TOFCAM635_IMAGE_DISTANCE_AMPLITUDE;
}

unsigned int TofCam635DistanceAmplitudeImage::getDistanceOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  //Make a pointer to the distance
  uint32_t *distanceAmplitude =  (uint32_t *)(&data.data()[dataOffset]);

  return (distanceAmplitude[index] & 0xFFFF);
}

unsigned int TofCam635DistanceAmplitudeImage::getAmplitudeOfPixel(const unsigned int index) const
{
  unsigned int dataOffset = CommunicationConstants::TofCam635Header::SIZE;

  //Make a pointer to the distance
  uint32_t *distanceAmplitude =  (uint32_t *)(&data.data()[dataOffset]);

  return (distanceAmplitude[index] >> 16);
}

}
