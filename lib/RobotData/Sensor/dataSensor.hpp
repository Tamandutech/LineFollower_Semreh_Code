#ifndef DATA_SENSOR_HPP
#define DATA_SENSOR_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "esp_log.h"

class dataSensor
{
public:
    dataSensor(std::string name = "dataVel");

    // Erro atual:
    DataAbstract<float> *Erro;
    

private:
    std::string name;
    const char *tag = "RobotData";

};

#endif