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
    TickType_t xLastTimeWake = xTaskGetTickCount();
    
    for(;;)
    {
        Robot::getInstance()->getVel()->EncMean->setData(CalculateRPM());

        int enc = Robot::getInstance()->getVel()->EncMean->getData();
        if(enc >= Robot::getInstance()->getMapeamento()->StopPoint->getData())
        {
            Robot::getInstance()->getStatus()->robotState->setData(CAR_STOPPED);
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



