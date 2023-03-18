#ifndef PID_SERVICE_HPP
#define PID_SERVICE_HPP

#include "thread.hpp"
#include "esp_log.h"
#include "ESP32Encoder.h"
#include "Injector/singleton.hpp"
#include "IOs.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"

using namespace cpp_freertos;

#define LEDC_TIMER              LEDC_TIMER_0 // Timer do LEDC utilizado
#define LEDC_MODE               LEDC_HIGH_SPEED_MODE // Modo de velocidade do LEDC
#define PWM_A_PIN               LEDC_CHANNEL_0 // Canal do LEDC utilizado
#define PWM_B_PIN               LEDC_CHANNEL_1 // Canal do LEDC utilizado
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Resolução do PWM
#define LEDC_FREQUENCY          5000 // Frequência em Hertz do sinal PWM

enum CarState
{
    CAR_IN_CURVE,
    CAR_IN_LINE,
    CAR_STOPPED,
    CAR_TUNING,
};

class PIDService : public Thread, public Singleton<PIDService>
{
public:
    PIDService(std::string name, uint32_t stackDepth, UBaseType_t priority);

    void Run() override;

private:

    float I = 0, P = 0, D = 0, PID = 0;
    float Ki = 0;
    float Kp = 0.0421;//0.0392
    float Kd = 0.0978; // 0.097
    float erro_anterior = 0;
    float erro_f = 0;
    float velesq = 0, veldir = 0;
    void ControleMotores(float PD, int vel_A, int vel_B);
    void AnalogWrite(ledc_channel_t channel, int pwm);
    void InitPWM(gpio_num_t pin, ledc_channel_t channel);
    float CalcularPID(float K_p, float K_d, float errof);
};

#endif