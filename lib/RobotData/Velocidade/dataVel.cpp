#include "dataVel.hpp"

dataVel::dataVel(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    EncRight = new DataAbstract<int32_t>("EncRight", name, 0);
    EncLeft = new DataAbstract<int32_t>("EncLeft", name, 0);
    EncMean = new DataAbstract<int32_t>("EncMean", name, 0);

    VelRight_inst = new DataAbstract<int16_t>("VelRight_inst", name, 0);
    VelLeft_inst = new DataAbstract<int16_t>("VelLeft_inst", name, 0);

    Base = new DataAbstract<int8_t>("Base", name, 0);
}