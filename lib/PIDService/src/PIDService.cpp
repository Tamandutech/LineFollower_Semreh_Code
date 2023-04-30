#include "PIDService.hpp"

PIDService::PIDService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    tag = name;
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);
    gpio_set_direction((gpio_num_t)in_dir1, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)in_dir2, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)in_esq1, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)in_esq2, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)stby, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)stby, 1);
    InitPWM((gpio_num_t)pwmA, PWM_A_PIN);
    InitPWM((gpio_num_t)pwmB, PWM_B_PIN);
}

void PIDService::Run()
{
    ESP_LOGE(tag.c_str(), "Inicio.");
    TickType_t xLastTimeWake = xTaskGetTickCount();
    auto get_sensor = Robot::getInstance()->getSensorArray();
    auto get_PID = Robot::getInstance()->getPID();
    auto get_Status = Robot::getInstance()->getStatus();
    auto get_Vel = Robot::getInstance()->getVel();
    ESP_LOGE(tag.c_str(), "Loop.");
    for(;;)
    {
        TrackState state = (TrackState) get_Status->robotState->getData();
        //ESP_LOGE("PID", "State: ");
        erro = get_sensor->Erro->getData();

        if(state == CAR_STOPPED){
            gpio_set_level((gpio_num_t)stby, 0);
            
        }else{
            Kp = get_PID->Kp(state)->getData();
            Kd = get_PID->Kd(state)->getData();

            PID = CalcularPID(Kp, Kd, erro);
            vel_base = get_Vel->VelBase(state)->getData();
            ControleMotores(PID, 150);
        }
        vTaskDelayUntil(&xLastTimeWake, 10 / portTICK_PERIOD_MS);
    }  
}

float PIDService::CalcularPID(float K_p, float K_d, float errof){
    float PID_now;
    P = errof;
    D = errof - erro_anterior;
    PID_now = (K_p * P) + (K_d * D);
    erro_anterior = errof;
    return PID_now;
}

void PIDService::ControleMotores(float PD, int vel_i){
    
    float velesq = vel_i + PD;
    float veldir = vel_i - PD;

    if (velesq < 0)
    {
        velesq = -1 * velesq;
        if(velesq > 255){
            velesq = 255;
        }
        gpio_set_level((gpio_num_t)in_dir1, 0);
        gpio_set_level((gpio_num_t)in_dir2, 1);
        AnalogWrite(PWM_A_PIN, velesq);
       
        
    }
    
    else if (veldir < 0)
    {
        if(veldir > 255){
            veldir = 255;
        }
        veldir = -1 * veldir;
        gpio_set_level((gpio_num_t)in_esq1, 1);
        gpio_set_level((gpio_num_t)in_esq2, 0);
        AnalogWrite(PWM_B_PIN, veldir);
    }else{
        if(veldir > 255){
            veldir = 255;
        }
        if(velesq > 255){
            velesq = 255;
        }

        gpio_set_level((gpio_num_t)in_dir1, 1);
        gpio_set_level((gpio_num_t)in_dir2, 0);
        AnalogWrite(PWM_A_PIN, velesq);

        gpio_set_level((gpio_num_t)in_esq1, 0);
        gpio_set_level((gpio_num_t)in_esq2, 1);
        AnalogWrite(PWM_B_PIN, veldir);

    }
}

void PIDService::AnalogWrite(ledc_channel_t channel, int pwm){
    ledc_set_duty_and_update(LEDC_MODE,channel,pwm,0); // Atribui um novo duty para o PWM

    if(cont_print >= 200)
    {
        ESP_LOGI(tag.c_str(), "Duty: %d",  ledc_get_duty(LEDC_MODE,PWM_A_PIN));
        cont_print = 0;
    }else
    {
        cont_print++;
    }
}

void PIDService::InitPWM(gpio_num_t pin, ledc_channel_t channel){
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode      = LEDC_MODE;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.timer_num       = LEDC_TIMER;
    ledc_timer.freq_hz         = LEDC_FREQUENCY; // Frequência de 5Khz
    ledc_timer.clk_cfg         = LEDC_AUTO_CLK; // Configuração da fonte de clock
    ledc_timer_config(&ledc_timer);

    // Prepara e aplica a configuração do canal do LEDC
    ledc_channel_config_t ledc_channel;
    ledc_channel.gpio_num       = pin;
    ledc_channel.speed_mode     = LEDC_MODE;
    ledc_channel.channel        = channel;
    ledc_channel.intr_type      = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel      = LEDC_TIMER;
    ledc_channel.duty           = 0; 
    ledc_channel.hpoint         = 0; // Ponto de início do duty cycle
    ledc_channel_config(&ledc_channel);

    ledc_fade_func_install(0);
}