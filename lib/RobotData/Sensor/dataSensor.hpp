#ifndef DATA_SENSOR_HPP
#define DATA_SENSOR_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "esp_log.h"

class dataStatus
{
public:
    dataSpeed(std::string name = "dataVel");

    // Contagem atual dos encoders
    DataAbstract<uint8_t> *robotState;

    DataAbstract<bool> *robotIsMapping;

    DataAbstract<bool> *encreading;

private:
    std::string name;
    const char *tag = "RobotData";

}

#endif