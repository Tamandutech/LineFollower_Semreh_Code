#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <variables.h>

#include <QTRSensors.h>

int enc = 0;

ESP32Encoder encoder;
ESP32Encoder encoder2;
QTRSensors sArray;

float Ki = 0;
float Kp = 0.0635;//0.065  M120    0.067  M130     0.059  ~M135   7.9V      0.07164  M155     0.0672  M130    | 0.067  M135  |
float Kd = 0.22; // 0.27           0.23            0.23                    0.2365            0.231           | 0.25   RCX   |

float KiR = 0;
float KpR = 0.047;//0.0392   0.059      0.0485  M255  | 0.0473  M255 |  0.044
float KdR = 0.23;//0.097                0.23          | 0.23    RCX  |/ 0.22

float KiZIG = 0;
float KpZIG = 0.024;
float KdZIG = 0.15;


float KpG = 0.0805;
float KdG = 0.25;

int calculate_rpm()
{

  enc_esq_pul = encoder.getCount() - pul_prev_eq;   // delta s
  enc_dir_pul = encoder2.getCount() - pul_prev_dir; // delta s

  pul_prev_eq = encoder.getCount();
  pul_prev_dir = encoder2.getCount();

  rpm_e = enc_esq_pul/0.015;
  rpm_d = enc_dir_pul/0.015;
  

  enc = 0 - (rpm_e - rpm_d);
  return enc;
  
}

void ler_sensores()
{

  uint16_t sArraychannels[sArray.getSensorCount()];
  erro_sensores = sArray.readLineWhite(sArraychannels) - 3500;
  //erro_sensores = sArray.readLineBlack(sArraychannels) - 3500;
  erro_f = -1 * erro_sensores;
 
}
void calcula_PID()
{
  
  P = erro_f;
  D = erro_f - erro_anterior;
  PID = (Kp * P) + (Kd * D);
  erro_anterior = erro_f;
}
void controle_motores(float vel_A, float vel_B)
{
  
  velesq = vel_A + PID;
  veldir = vel_B - PID;

  if (velesq < 0)
  {
    velesq = -1 * velesq;
    if(velesq > 255){
      velesq = 255;
    }
    digitalWrite(in_dir1, LOW);
    digitalWrite(in_dir2, HIGH);
    analogWrite(pwmA, velesq);
  }
  
  else if (veldir < 0)
  {
    if(veldir > 255){
      veldir = 255;
    }
    veldir = -1 * veldir;
    digitalWrite(in_esq1, HIGH);
    digitalWrite(in_esq2, LOW);
    analogWrite(pwmB, veldir);
  }else{
    if(veldir > 255){
      veldir = 255;
    }
    if(velesq > 255){
      velesq = 255;
    }

    digitalWrite(in_dir1, HIGH);
    digitalWrite(in_dir2, LOW);
    analogWrite(pwmA, velesq);

    digitalWrite(in_esq1, LOW);
    digitalWrite(in_esq2, HIGH);
    analogWrite(pwmB, veldir);

  }

}


void calcula_PID_R()
{
  
  PR = erro_f;
  DR = erro_f - erro_anterior;
  PIDR = (KpR * PR) + (KdR * DR);
  erro_anterior = erro_f;
}


void controle_motores_R(float vel_AR, float vel_BR){
  velesqR = vel_AR + PIDR;
  veldirR = vel_BR - PIDR;

  if (velesqR < 0)
  {
    velesqR = -1 * velesqR;
    if(velesqR > 255){
      velesqR = 255;
    }
    digitalWrite(in_dir1, LOW);
    digitalWrite(in_dir2, HIGH);
    analogWrite(pwmA, velesqR);
  }
  
  else if (veldirR < 0)
  {
    if(veldirR > 255){
      veldirR = 255;
    }
    veldirR = -1 * veldirR;
    digitalWrite(in_esq1, HIGH);
    digitalWrite(in_esq2, LOW);
    analogWrite(pwmB, veldirR);
  }else{
    if(veldirR > 255){
      veldirR = 255;
    }
    if(velesqR > 255){
      velesqR = 255;
    }

    digitalWrite(in_dir1, HIGH);
    digitalWrite(in_dir2, LOW);
    analogWrite(pwmA, velesqR);

    digitalWrite(in_esq1, LOW);
    digitalWrite(in_esq2, HIGH);
    analogWrite(pwmB, veldirR);

  }
}

void calcula_PID_zig()
{
  
  PZIG = erro_f;
  DZIG = erro_f- erro_anterior;
  PIDZIG = (KpZIG * PZIG) + (KdZIG * DZIG);
  erro_anterior = erro_f;
}
void controle_motores_zig(float vel_AR, float vel_BR){
  velesqZIG = vel_AR + PIDZIG;
  veldirZIG = vel_BR - PIDZIG;

  if (velesqZIG < 0)
  {
    velesqZIG = -1 * velesqZIG;
    if(velesqZIG > 255){
      velesqZIG = 255;
    }
    digitalWrite(in_dir1, LOW);
    digitalWrite(in_dir2, HIGH);
    analogWrite(pwmA, velesqZIG);
  }
  
  else if (veldirZIG < 0)
  {
    if(veldirZIG > 255){
      veldirZIG = 255;
    }
    veldirZIG = -1 * veldirZIG;
    digitalWrite(in_esq1, HIGH);
    digitalWrite(in_esq2, LOW);
    analogWrite(pwmB, veldirZIG);
  }else{
    if(veldirZIG > 255){
      veldirZIG = 255;
    }
    if(velesqZIG > 255){
      velesqZIG = 255;
    }

    digitalWrite(in_dir1, HIGH);
    digitalWrite(in_dir2, LOW);
    analogWrite(pwmA, velesqZIG);

    digitalWrite(in_esq1, LOW);
    digitalWrite(in_esq2, HIGH);
    analogWrite(pwmB, veldirZIG);

  }
}


bool ler_sens_lat()
{
  #define tempoDebounce 200

  bool estadoSLatEsq;
  static bool estadoSLatEsqAnt;
  static bool estadoRet = true;
  static unsigned long delaySenLat = 0;
  int x = 0;
  int y = 0;
  if((millis() - delaySenLat)> tempoDebounce){
    x = analogRead(s_lat_esq);
    y = analogRead(s_lat_dir);
    // if(x < 100  && y <100){
    //   estadoSLatEsq = false;     
    // }
    //Serial.println(x);
    if(x < 2500){
      estadoSLatEsq = true;     
    }
    else{
      estadoSLatEsq = false;
    }
    
    
    
    if(estadoSLatEsq && (estadoSLatEsq != estadoSLatEsqAnt)){
      estadoRet = !estadoRet;
      delaySenLat = millis();
    }
    estadoSLatEsqAnt = estadoSLatEsq;
    
  }
  return estadoSLatEsq;
}



void controle_sem_mapeamento(){ 

        calcula_PID();
        controle_motores(120,120);
        
}


void controle_com_mapeamento(int encVal){

    if((encVal > 1300) && (encVal < 2300)){ // primeira curva
        calcula_PID();
        controle_motores(105,105);
      }
     else if((encVal > 17500) && (encVal < 18500)){ //Primeira reta
        calcula_PID_R();
        controle_motores_R(220,220);
      }
      else if((encVal > 20900) && (encVal < 21500)){ //Segunda Reta
        calcula_PID_R();
        controle_motores_R(180,180);
      }
      else if((encVal > 22600) && (encVal < 23600)){ 
        calcula_PID();
        controle_motores(125,125);
      }
      else if((encVal > 24000) && (encVal < 30500)){ // trevo
        calcula_PID_R();
        controle_motores_R(170,170);
      }
      else if((encVal > 31500) && (encVal < 33000)){ 
        calcula_PID();
        controle_motores(120,120);
      }
      else if((encVal > 33000) && (encVal < 34800)){ // curvinha
        calcula_PID_R();
        controle_motores_R(170,170);
      }

      else if((encVal > 39000) && (encVal < 39800)){ //Reta Antes Zig
        calcula_PID_R();
        controle_motores_R(220,220);
      }
      else if((encVal > 40700) && (encVal < 43500)){ //ZigZag
        calcula_PID();
        controle_motores(125,125);
      }
    else if((encVal > 43700) && (encVal < 45000)){ //Reta Depois Zig
        calcula_PID_R();
        controle_motores_R(240,240);
      }
    else if((encVal > 46700) && (encVal < 54700)){ //Rotatória 1
        calcula_PID();
        controle_motores(200,200);
      }
    else if((encVal > 56500) && (encVal < 61500)){ //Rotatória 2
        calcula_PID();
        controle_motores(200,200);
      }
    else if((encVal > 63500) && (encVal < 65000)){ //Rotatória 3
        calcula_PID();
        controle_motores(190,190);
      }
    else if((encVal > 69000) && (encVal < 76000)){ //Reta Giga
        calcula_PID_R();
        controle_motores_R(245,245);
      }
      
    else if((encVal > 79500) && (encVal < 79900)){ //Reta Giga
        calcula_PID();
        controle_motores(60,60);
      }
    else if(encVal > 79900){
        digitalWrite(stby, LOW);
      }
    else{
        calcula_PID();
        controle_motores(140,140);
      }

  }

void controle_com_mapeamento_teste(int encVal){

    if((encVal < 2000)){
      calcula_PID();
        controle_motores(160,160);
    }

    else{
        calcula_PID();
        controle_motores(140,140);
      }

  }

int v = 0;
void mapeamento(){
  timer_in = millis();

  digitalWrite(led, LOW);
  digitalWrite(buzzer, LOW);
        
  if(ler_sens_lat() == true){  
        if(timer_in - timer_prev2 >= 15){
          v = (v+1);
          Serial.print(v);
          Serial.print(" ");
          Serial.println((encoder.getCount() + encoder.getCount())/2);
          
          //Serial.println(timer_in);

        }
      digitalWrite(led, HIGH); 
      digitalWrite(buzzer, HIGH);
       timer_prev2 = timer_in;
     }
    
}
void setup()
{
  Serial.begin(9600);
  
 

  pinMode(in_dir1, OUTPUT);
  pinMode(in_dir2, OUTPUT);
  pinMode(in_esq1, OUTPUT);
  pinMode(in_esq2, OUTPUT);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(stby, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(s_lat_esq, INPUT);
  pinMode(s_lat_dir, INPUT);
  pinMode(buzzer, OUTPUT);

  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoder.attachFullQuad(enc_eq_B, enc_eq_A);
  encoder2.attachFullQuad(enc_dir_A, enc_dir_B);

  digitalWrite(stby, HIGH);

  encoder.clearCount();
  encoder2.clearCount();

  sArray.setTypeMCP3008();
  sArray.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7}, 8, (gpio_num_t)out_s_front, (gpio_num_t)in_s_front, (gpio_num_t)clk, (gpio_num_t)cs_s_front, 1350000, VSPI_HOST);
  sArray.setSamplesPerSensor(5);

  for (uint16_t i = 0; i < 300; i++)
  {
    sArray.calibrate();
    delay(20);
  }
  

  //Blynk.run();
  
}
bool bly = false;

void loop()
{
     timer_in = millis();

    //if(timer_in - timer_prev2 >= 180000){

      if(timer_in - timer_prev3 >= 15){
        ler_sensores();
        encVal = ((encoder.getCount() + encoder2.getCount())/2);
        //mapeamento();
        //Serial.println(encVal);
        controle_sem_mapeamento();
        //controle_com_mapeamento(encVal);
        timer_prev3 = timer_in;

      } 
      //timer_prev2 = timer_in;
    //}           
}

