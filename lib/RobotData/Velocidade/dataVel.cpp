#include "dataVel.hpp"

dataVel::dataVel(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    EncMean = new DataAbstract<int32_t>("EncMean", name, 0);

    Base = new DataAbstract<int8_t>("Base", name, 0);
}