#include <robo.h>

//left speed
int robo::get_speed_left(){
    return speed_left;
}

void robo::set_speed_left(int speed_left){
    this->speed_left = speed_left;
}

//right speed
int robo::get_speed_right(){
    return speed_right;
}

void robo::set_speed_right(int speed_right){
    this->speed_right = speed_right;
}

//number of left marks
int robo::get_mark_left(){
    return mark_left;
}

void robo::set_mark_left(int mark_left){
    this->mark_left = mark_left;
}

//number of right marks
int robo::get_mark_right(){
    return mark_right;
}

void robo::set_mark_right(int mark_right){
    this->mark_right = mark_right;
}

//bool for left marks
bool robo::get_sensor_left(){
    return sensor_left;
}

void robo::set_sensor_left(int sensor_left){
    this->sensor_left = sensor_left;
}

//bool for right marks
bool robo::get_sensor_right(){
    return sensor_right;
}

void robo::set_sensor_right(int sensor_right){
    this->sensor_right = sensor_right;
}

//sensor's array
int* robo::get_array(){
    return this->array;
}

void robo::set_array(int *array){
    this->array = array;
}

//posicao (erro)
int robo::get_posicao(){
    return posicao;
}

void robo::set_posicao(int posicao){
    this->posicao = posicao;
}

//left encoder data
int robo::get_encoder_left(){
    return encoder_left;
}

void robo::set_encoder_left(int encoder_left){
    this->encoder_left = encoder_left;
}

//right encoder data
int robo::get_encoder_right(){
    return encoder_right;
}

void robo::set_encoder_right(int encoder_right){
    this->encoder_right = encoder_right;
} 

