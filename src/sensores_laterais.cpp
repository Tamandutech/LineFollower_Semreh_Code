#include <setup.h>
#include <Arduino.h>

unsigned int sensor_direita = 0;
unsigned int contador_sensor_direita = 0;
unsigned int contador_sensor_esquerda = 0; 
unsigned int sensor_esquerda  = 0;
boolean estado_sensor_esquerdo = true;
boolean estado_sensor_direito = true;

void sensores_laterais (){
    sensor_direita  = analogRead(sensor_lateral_direito);
    sensor_esquerda = analogRead(sensor_lateral_esquerdo);
    //sensor direito
    if(sensor_direita < indicador_lateral_direito && estado_sensor_direito == true){
        estado_sensor_direito = false;
        contador_sensor_direita++;

    }
    else{
        estado_sensor_direito = true;
    }
    
    // sensor esquerdp
     if(sensor_esquerda < indicador_lateral_esquerdo && estado_sensor_esquerdo == true){
        estado_sensor_esquerdo = false;
        contador_sensor_esquerda++;

    }
    else{
        estado_sensor_esquerdo = true;
    }


}