#include <setup.h>
#include <Arduino.h>
#include <sensores_laterais.cpp>

void parada (){
    if(contador_sensor_esquerda >= 'curvas_esquerda' || contador_sensor_direita>='qtde_curvas' ){
            digitalWrite('ain1',LOW );
            analogWrite('pwm_direito_a', 0);
            digitalWrite('bin1',LOW );
            analogWrite('pwm_esquerdo_b', 0);
            delay(10000);
    }
    
}