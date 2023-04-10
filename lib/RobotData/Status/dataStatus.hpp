#ifndef DATA_STATUS_HPP
#define DATA_STATUS_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "esp_log.h"

class dataStatus
{
public:
    dataStatus(std::string name = "dataVel");

    DataAbstract<uint8_t> *robotState;

    DataAbstract<bool> *robotIsMapping;

    DataAbstract<bool> *encreading;

private:
    std::string name;
    const char *tag = "RobotData";

};

#endif