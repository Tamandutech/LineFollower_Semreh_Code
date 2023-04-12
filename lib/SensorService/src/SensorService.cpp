#include "SensorService.hpp"

SensorService::SensorService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    // Iniciando os sensores frontais:
    sArray.setTypeMCP3008();
    sArray.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7}, 8, (gpio_num_t)out_s_front, (gpio_num_t)in_s_front, (gpio_num_t)clk, (gpio_num_t)cs_s_front, 1350000, VSPI_HOST);
    sArray.setSamplesPerSensor(5);
    
    // Iniciando os sensores laterais:
    adc1_config_width(ADC_WIDTH_10Bit);
    adc1_config_channel_atten(s_lat_dir, ADC_ATTEN_11db);
    adc2_config_channel_atten(s_lat_esq, ADC_ATTEN_11db);

    Calibracao();
}

void SensorService::Run()
{
    TickType_t xLastTimeWake = xTaskGetTickCount();
    auto get_Map = Robot::getInstance()->getMapeamento();
    auto get_Vel = Robot::getInstance()->getVel();
    for(;;)
    {
        LerSensores();
        count++;
        if(count >= 200){
            if(LerLateral()){
                get_Map->leftPassedInc();
                int left_marks = get_Map->leftMarks->getData();
                int enc = get_Vel->EncMean->getData();
                ESP_LOGI(GetName().c_str(), "%dº - Média encoders: %d",left_marks, enc);
            }
            count = 0; 
        }

        vTaskDelayUntil(&xLastTimeWake, 10 / portTICK_PERIOD_MS);

    }
    
}

void SensorService::LerSensores(){
    uint16_t sArraychannels[sArray.getSensorCount()];
    erro_f = -1 * sArray.readLineWhite(sArraychannels) - 3500;
    Robot::getInstance()->getSensorArray()->Erro->setData(erro_f);
}

bool SensorService::LerLateral(){
    bool estadoSLatEsq;
    int x = 0;

    adc2_get_raw(s_lat_esq, ADC_WIDTH_10Bit, &x);

    if(x < 150){
      estadoSLatEsq = true;     
    }
    else{
      estadoSLatEsq = false;
    }
    
    return estadoSLatEsq;
}

void SensorService::Calibracao(){
    for (uint16_t i = 0; i < 300; i++)
    {
        sArray.calibrate();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}