#include "dataVel.hpp"

dataVel::dataVel(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    EncMean = new DataAbstract<int32_t>("EncMean", name, 0);

    Base_line = new DataAbstract<int8_t>("Base_line", name, 0);
    Base_curve = new DataAbstract<int8_t>("Base_curve", name, 0);
    Base_zigzag = new DataAbstract<int8_t>("Base_zigzag", name, 0);
    Base_mapping = new DataAbstract<int8_t>("Base_mapping", name, 0);
}

DataAbstract<int8_t> *dataVel::VelBase(TrackState state)
{
    if(state == LINE){
        return Base_line;
    }
    else if(state == CURVE){
        return Base_curve;
    }
    else if(state == ZIGZAG){
        return Base_zigzag;
    }
    else if(state == MAPPING){
        return Base_mapping;
    }
    ESP_LOGE(tag, "Estado do Robô ou Objeto Vel. Base inválido para esse método: %s:%d para obter a Vel. Base, retornando valor null.", name.c_str(),state);
    return nullptr;
}