#include "dataSensor.hpp"

dataSensor::dataSensor(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    Erro = new DataAbstract<float>("Erro", name);
    
}