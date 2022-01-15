#include <includes.h>
void configuracao_pinos (){
    pinMode(sensor_lateral_esquerdo,INPUT)
    pinMode(sensor_lateral_direito,INPUT);
    pinMode(sensores_array,INPUT);
    pinMode(pwm_direito_a,OUTPUT);
    pinMode(pwm_esquerdo_b,OUTPUT);
    pinMode(ain1,OUTPUT);
    pinMode(ain2,OUTPUT);
    pinMode(pwm_esquerdo_b,OUTPUT);
    pinMode(b1n1,OUTPUT);
    pinMode(b1n2,OUTPUT);
    pinMode(encoder_motor_dir_a,INPUT);
    pinMode(encoder_motor_dir_b,INPUT);
    pinMode(encoder_motor_esq_a,INPUT);
    pinMode(encoder_motor_esq_b,INPUT);
}