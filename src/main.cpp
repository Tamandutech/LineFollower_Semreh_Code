#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "SensorService.hpp"
#include "PIDService.hpp"
#include "RPMService.hpp"

#include "esp_log.h"

// Criando os objetos dos serviços:
SensorService *sensorService;
RPMService *rpmService;
PIDService *pidService;

extern "C"
{
  void app_main(void);
}

void app_main() 
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("main", ESP_LOG_INFO);

    sensorService = new SensorService("SensorService", 4096, 5);
    sensorService->Start();

    rpmService = new RPMService("RPMService", 4096, 4);
    rpmService->Start();
    
    pidService = new PIDService("PIDService", 4096, 6);
    pidService->Start();

    for(;;)
    {
        ESP_LOGI("main", "RPMService: %d", eTaskGetState(rpmService->GetHandle()));
        ESP_LOGI("main", "PIDService: %d", eTaskGetState(pidService->GetHandle()));
        ESP_LOGI("main", "SensorService: %d", eTaskGetState(sensorService->GetHandle()));

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}