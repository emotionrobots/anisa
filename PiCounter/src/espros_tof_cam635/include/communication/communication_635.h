/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup communication_epc635 EPC635
 * @brief Implementation for Cam635
 * @ingroup communication
 *
 * @{
 */
#ifndef COMMUNICATION_635_H
#define COMMUNICATION_635_H

#include "communication.h"
#include "tof_cam635_image.h"
#include "tof_cam635_distance_image.h"
#include "tof_cam635_distance_amplitude_image.h"
#include "tof_cam635_distance_grayscale_image.h"
#include "tof_cam635_grayscale_image.h"


namespace ComLib
{
//! Communication Implementation for TofCam635
/*!
 * This class implements the specific functionality for TofCam635 device.
 */
class Communication635: public Communication
{
public:
    Communication635();
    ErrorNumber_e setMode(const unsigned int mode);

    //signals
    boost::signals2::signal<void (std::shared_ptr<TofCam635Image>)> sigReceivedDistance;
    boost::signals2::signal<void (std::shared_ptr<TofCam635Image>)> sigReceivedGrayscale;
    boost::signals2::signal<void (std::shared_ptr<TofCam635Image>)> sigReceivedDistanceGrayscale;
    boost::signals2::signal<void (std::shared_ptr<TofCam635Image>)> sigReceivedDistanceAmplitude;
    boost::signals2::signal<void (std::vector<uint8_t>)> sigReceivedLensCalibrationData;

private:

    unsigned int getBaudRate();    
    bool setupDevice(const Device_e device);
    void processDistanceAmplitude(const std::vector<uint8_t> &array);
    void processDistance(const std::vector<uint8_t> &array);
    void processGrayscale(const std::vector<uint8_t> &array);
    void processDistanceGrayscale(const std::vector<uint8_t> &array);
    void processLensCalibrationData(const std::vector<uint8_t> &array);
    //Mode_e actualMode;   ///<Stores the actual selected mode
};
}

#endif // COMMUNICATION_635_H

/** @} */
