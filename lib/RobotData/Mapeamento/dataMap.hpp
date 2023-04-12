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

    // Ponto de parada (mesma medida do encoder)
    DataAbstract<int32_t> *StopPoint;

    // Incrementa a contagem de marcas da lateral esquerda
    void leftPassedInc();
    // Incrementa a contagem de marcas da lateral direita
    void rightPassedInc();

private:
    std::string name;
    const char *tag = "RobotData";

};

#endif