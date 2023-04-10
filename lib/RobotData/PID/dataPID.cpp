#include "dataPID.hpp"

dataPID::dataPID(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(tag, "Criando objeto: %s (%p)", name.c_str(), this);

    Kp_line = new DataAbstract<float>("Kp_line", name);
    Kd_line = new DataAbstract<float>("Kd_line", name);

    Kp_curve = new DataAbstract<float>("Kp_curve", name);
    Kd_curve = new DataAbstract<float>("Kd_curve", name);

    Kp_zigzag = new DataAbstract<float>("Kp_zigzag", name);
    Kd_zigzag = new DataAbstract<float>("Kd_zigzag", name);

    Kp_mapping = new DataAbstract<float>("Kp_tunning", name);
    Kd_mapping = new DataAbstract<float>("Kd_tunning", name);
}

DataAbstract<float> *dataPID::Kp(TrackState state)
{
    if(state == LINE){
        return Kp_line;
    }
    else if(state == CURVE){
        return Kp_curve;
    }
    else if(state == ZIGZAG){
        return Kp_zigzag;
    }
    else if(state == MAPPING){
        return Kp_mapping;
    }
    ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kp do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}

DataAbstract<float> *dataPID::Kd(TrackState state)
{
    if(state == LINE){
        return Kd_line;
    }
    else if(state == CURVE){
        return Kd_curve;
    }
    else if(state == ZIGZAG){
        return Kd_zigzag;
    }
    else if(state == MAPPING){
        return Kd_mapping;
    }
    ESP_LOGE(tag, "Estado do Robô ou Objeto PID inválido para esse método: %s:%d para obter o Kd do PID, retornando valor null.", name.c_str(),state);
    return nullptr;
}