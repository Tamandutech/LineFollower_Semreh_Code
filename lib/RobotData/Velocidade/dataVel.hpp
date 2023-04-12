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
    DataAbstract<int32_t> *EncMean;

    // Velocidade base:
    DataAbstract<int8_t> *Base;



private:
    std::string name;
    const char *tag = "RobotData";

};

#endif