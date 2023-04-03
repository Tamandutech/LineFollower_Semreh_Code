#ifndef DATA_VEL_HPP
#define DATA_VEL_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "esp_log.h"

class dataVel
{
public:
    dataVel(std::string name = "dataVel");

    // Contagem atual dos encoders
    DataAbstract<int32_t> *EncRight;
    DataAbstract<int32_t> *EncLeft;
    DataAbstract<int32_t> *EncMean;

    // Velocidade atual:
    DataAbstract<int16_t> *VelRight_inst;
    DataAbstract<int16_t> *VelLeft_inst;

    // Velocidade base:
    DataAbstract<int8_t> *Base;



private:
    std::string name;
    const char *tag = "RobotData";

};

#endif