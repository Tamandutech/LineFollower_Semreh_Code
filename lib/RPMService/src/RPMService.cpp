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
    for(;;)
    {
        this->Suspend();
        CalculateRPM();
        vTaskDelay(0);
    }
    
}

int RPMService::CalculateRPM(){
    enc_esq_pul = encoder.getCount() - pul_prev_eq;   // delta s
    enc_dir_pul = encoder2.getCount() - pul_prev_dir; // delta s

    pul_prev_eq = encoder.getCount();
    pul_prev_dir = encoder2.getCount();

    enc = (enc_esq_pul + enc_dir_pul) /2;
    ESP_LOGI(GetName().c_str(), "Média encoders: %d", enc);
    return enc;
}

