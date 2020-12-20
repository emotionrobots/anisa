/**
 * Copyright (C) 2018 Espros Photonics Corporation
 *
 * @defgroup production_information_helper Production Information Helper
 * @brief Helper class for the  Production Information
 * @ingroup communication
 *
 * @{
 */
#include "production_information_helper.h"

namespace ComLib
{

ProductionInformationHelper::ProductionInformationHelper()
{
    year = 0;
    week = 0;
}

unsigned int ProductionInformationHelper::getYear()
{
    return year;
}

unsigned int ProductionInformationHelper::getWeek()
{
    return week;
}

void ProductionInformationHelper::onReceivedProductionInformation(const uint8_t year, const uint8_t week)
{
    this->year = year;
    this->week = week;
}

}
