#ifndef DATA_PID_HPP
#define DATA_PID_HPP

#include <stdint.h>
#include <stddef.h>
#include <string>

#include "DataAbstract.hpp"

#include "dataEnums.hpp"

#include "esp_log.h"

class dataPID
{
public:
    dataPID(std::string name = "dataVel");

    // Constantes do PID definidas pelo trecho da pista
    DataAbstract<float> *Kp(TrackState state);
    DataAbstract<float> *Kd(TrackState state);

private:
    std::string name;
    const char *tag = "RobotData";

    // Parâmetros do PID  
    DataAbstract<float> *Kp_line; // salvar
    DataAbstract<float> *Kd_line; // salvar

    DataAbstract<float> *Kp_curve; // salvar
    DataAbstract<float> *Kd_curve; // salvar

    DataAbstract<float> *Kp_zigzag; // salvar
    DataAbstract<float> *Kd_zigzag; // salvar
    
    DataAbstract<float> *Kp_mapping; // salvar
    DataAbstract<float> *Kd_mapping; // salvar


};

#endif