#ifndef DATA_VEL_HPP
#define DATA_VEL_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "dataEnums.hpp"

#include "esp_log.h"

class dataVel
{
public:
    dataVel(std::string name = "dataVel");

    // Contagem atual dos encoders
    DataAbstract<int32_t> *EncMean;

    DataAbstract<int8_t> *VelBase(TrackState state);

private:
    std::string name;
    const char *tag = "RobotData";

    // Velocidade base:
    DataAbstract<int8_t> *Base_line;
    DataAbstract<int8_t> *Base_curve;
    DataAbstract<int8_t> *Base_zigzag;
    DataAbstract<int8_t> *Base_mapping;

};

#endif