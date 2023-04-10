#ifndef DATA_MAP_HPP
#define DATA_MAP_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"
#include "DataMap.hpp"

#include "esp_log.h"

class dataMap
{
public:
    dataMap(std::string name = "dataVel");

    // Quantidade atual de marcas da lateral esquerda
    DataAbstract<uint16_t> *leftMarks;
    // Quantidade atual de marcas da lateral direita
    DataAbstract<uint16_t> *rightMarks;

private:
    std::string name;
    const char *tag = "RobotData";

};

#endif