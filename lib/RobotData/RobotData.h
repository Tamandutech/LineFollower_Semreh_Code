#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include <stdint.h>
#include <stddef.h>
#include <string>
#include <queue>
#include <atomic>
#include <mutex>

#include "Velocidade/dataVel.hpp"
#include "Status/dataStatus.hpp"
#include "Sensor/dataSensor.hpp"
#include "PID/dataPID.hpp"
#include "Mapeamento/dataMap.hpp"
#include "dataEnums.hpp"

#include "esp_log.h"

class Robot
{
public:
    static Robot *getInstance(std::string name = "RobotData")
    {
        Robot *sin = instance.load(std::memory_order_acquire);
        if (!sin)
        {
            std::lock_guard<std::mutex> myLock(instanceMutex);
            sin = instance.load(std::memory_order_relaxed);
            if (!sin)
            {
                sin = new Robot(name);
                instance.store(sin, std::memory_order_release);
            }
        }

        return sin;
    };

    dataVel *getVel();
    dataStatus *getStatus();
    dataSensor *getSensorArray();
    dataSensor *getSlatArray();
    dataPID *getPID();
    dataMap *getMapeamento();

    std::string GetName();

private:
    std::string name;

    static std::atomic<Robot *> instance;
    static std::mutex instanceMutex;

    dataVel *velocidade;
    dataStatus *Status;
    dataSensor *SensorArray;
    dataSensor *sLatArray;
    dataPID *PID;
    dataMap *Mapeamento;

    Robot(std::string name);
};

#endif