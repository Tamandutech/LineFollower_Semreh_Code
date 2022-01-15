#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>
#include <Esp32Encoder.h>
#include <setup.h>

#include <ctrl_pid.h>


CONFIG config;

PID pid(config.pid.Kp, config.pid.Ki, config.pid.Kd, 
        config.pid.P, config.pid.I, config.pid.D,
        config.pid.erro, config.pid.erro_ant); //criacao do objeto PID (ta com problema)



void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}


//como programar o motor a partir disso: 
//se PID > 0, a velocidade do motor da direita sera a max permitida - PID
//se PID < 0, a velocidade do motor da esquerda sera a max permitida + PID