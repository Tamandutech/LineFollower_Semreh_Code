#ifndef SENSOR_SERVICE_HPP
#define SENSOR_SERVICE_HPP

#include "thread.hpp"
#include "esp_log.h"
#include "Injector/singleton.hpp"
#include "QTRSensors.h"
#include "IOs.hpp"
#include "RPMService.hpp"

using namespace cpp_freertos;

class SensorService : public Thread, public Singleton<SensorService>
{
public:
    SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;

private:

    int count = 0;
    int erro_f = 0;
    QTRSensors sArray;
    void LerSensores();
    void Calibracao();
    bool LerLateral();
};

#endif