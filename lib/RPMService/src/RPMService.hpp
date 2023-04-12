#ifndef RPM_SERVICE_HPP
#define RPM_SERVICE_HPP

#include "thread.hpp"
#include "esp_log.h"
#include "ESP32Encoder.h"
#include "Injector/singleton.hpp"
#include "IOs.hpp"
#include "RobotData.h"

using namespace cpp_freertos;

class RPMService : public Thread, public Singleton<RPMService>
{
public:
    RPMService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;

private:

    long int enc_esq_pul;
    long int enc_dir_pul;
    long int pul_prev_eq = 0;
    long int pul_prev_dir = 0;
    int enc = 0;
    ESP32Encoder encoder;
    ESP32Encoder encoder2;
    int CalculateRPM();
};

#endif