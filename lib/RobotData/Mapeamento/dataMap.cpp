#include "dataMap.hpp"

dataMap::dataMap(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    leftMarks = new DataAbstract<uint16_t>("leftMarks", name, 0);
    rightMarks = new DataAbstract<uint16_t>("rightMarks", name, 0);

    StopPoint = new DataAbstract<int32_t>("StopPoint", name, 0);
    
}

void dataMap::leftPassedInc()
{
    this->leftMarks->setData(this->leftMarks->getData() + 1);
}

void dataMap::rightPassedInc()
{
    this->rightMarks->setData(this->rightMarks->getData() + 1);
}