#include "RPMService.hpp"

RPMService::RPMService(std::string name, uint32_t stackDepth, UBaseType_t priority):Thread(name, stackDepth, priority)
{
    esp_log_level_set(name.c_str(), ESP_LOG_INFO);

    encoder.attachFullQuad(enc_eq_B, enc_eq_A);
    encoder2.attachFullQuad(enc_dir_A, enc_dir_B);
    encoder.clearCount();
    encoder2.clearCount();
}

void RPMService::Run()
{
    ESP_LOGE("RPM", "Inicio.");
    TickType_t xLastTimeWake = xTaskGetTickCount();
    auto get_Vel = Robot::getInstance()->getVel();
    auto get_Map = Robot::getInstance()->getMapeamento();
    auto get_Status = Robot::getInstance()->getStatus();
    ESP_LOGE("RPM", "Loop.");
    for(;;)
    {
        get_Vel->EncMean->setData(CalculateRPM());

        int enc = get_Vel->EncMean->getData();
        if(enc >= get_Map->StopPoint->getData())
        {
            get_Status->robotState->setData(CAR_STOPPED);
        }
        else{
            get_Status->robotState->setData(LINE);
        }
        

        vTaskDelayUntil(&xLastTimeWake, 10 / portTICK_PERIOD_MS);
    }
    
}

int RPMService::CalculateRPM(){
    enc_esq_pul = encoder.getCount() - pul_prev_eq;   // delta s
    enc_dir_pul = encoder2.getCount() - pul_prev_dir; // delta s

    pul_prev_eq = encoder.getCount();
    pul_prev_dir = encoder2.getCount();

    enc = (enc_esq_pul + enc_dir_pul) /2;
    
    return enc;
}



