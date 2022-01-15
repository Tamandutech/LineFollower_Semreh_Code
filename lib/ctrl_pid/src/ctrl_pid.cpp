#include <ctrl_pid.h>

PID::PID(float Kp, float Ki, float Kd, float P, float I, float D, float erro, float erro_ant){
    float _Kp = Kp;
    float _Ki = Ki;
    float _Kd = Kd;

    this->P = P;
    float _I = I; //sempre vai ser 0
    float _D = D;

    float _erro = erro;
    float _erro_ant = erro_ant;
}

float PID::calculo_pid(float _Kp, float _Ki, float _Kd, float P, float _I, float _D, float _erro, float _erro_ant){

    if(_erro == 0){
        _I = 0;
    }

    P = _erro;
    _I += _erro;

    if(_I > 255){
        _I = 255;
    }
    else if(_I < -255){
        _I = -255;
    }

    _D = _erro - _erro_ant;
    _PID = _Kp*P + _Ki*_I + _Kd*_D;

    _erro_ant = _erro;

    return _PID;
} 