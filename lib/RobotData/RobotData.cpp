#include "RobotData.h"

std::atomic<Robot *> Robot::instance;
std::mutex Robot::instanceMutex;

Robot::Robot(std::string name)
{
    // Definindo nome do objeto, para uso nas logs do componente.
    this->name = name;
    ESP_LOGD(name.c_str(), "Criando objeto: %s (%p)", name.c_str(), this);


    // Instânciando objetos componentes do Robô.
    ESP_LOGD(name.c_str(), "Criando sub-objetos para o %s", "Robô");

    this->velocidade = new dataVel("velocidade");
    ESP_LOGD(name.c_str(), "velocidade (%p)", this->velocidade);

    this->Status = new dataStatus("Status");
    ESP_LOGD(name.c_str(), "Status (%p)", this->Status);

    this->SensorArray = new dataSensor("SensorArray");
    ESP_LOGD(name.c_str(), "SensorArray (%p)", this->SensorArray);

    this->sLatArray = new dataSensor("sLatArray");
    ESP_LOGD(name.c_str(), "sLatArray (%p)", this->sLatArray);

    this->PID = new dataPID("PID");
    ESP_LOGD(name.c_str(), "PID (%p)", this->PID);

    this->Mapeamento = new dataMap("Mapeamento");
    ESP_LOGD(name.c_str(), "Mapeamento (%p)", this->Mapeamento);
}

dataVel *Robot::getVel()
{
    return this->velocidade;
}

dataStatus *Robot::getStatus()
{
    return this->Status;
}

dataSensor *Robot::getSensorArray()
{
    return this->SensorArray;
}

dataSensor *Robot::getSlatArray()
{
    return this->sLatArray;
}

dataPID *Robot::getPID()
{
    return this->PID;
}

dataMap *Robot::getMapeamento()
{
    return this->Mapeamento;
}

std::string Robot::GetName()
{
    return this->name;
}