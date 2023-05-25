#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <variables.h>

#include <QTRSensors.h>

int enc = 0;

ESP32Encoder encoder;
ESP32Encoder encoder2;
QTRSensors sArray;
QTRSensors sLat;

float Kp = 0.063;//0.065  M120    0.067  M130     0.059  ~M135   7.9V      0.07164  M155     0.0672  M130    | 0.067  M135  |
float Kd = 0.27; // 0.27           0.23            0.23                    0.2365            0.231           | 0.25   RCX   |

float KpReta = 0.047;//0.0392   0.059      0.0485  M255  | 0.0473  M255 |  0.044
float KdReta = 0.23;// 0.097                0.23          | 0.23    RCX  |/ 0.22

float KpCurva = 0.0735;
float KdCurva = 0.27;

float KpZIG = 0.0735;
float KdZIG = 0.27;

void mudar_vel(int velocidade, int lado_motor, int direcao, uint8_t in_pin1, uint8_t in_pin2, uint8_t pwm){
  // Funcao que muda a velocidade de um motor por vez
  if(((lado_motor == DIREITO)&&(direcao == FRENTE)) || ((lado_motor == ESQUERDO)&&(direcao == REH)))
  {
    digitalWrite(in_pin1, HIGH);
    digitalWrite(in_pin2, LOW);
  }
  else
  {
    digitalWrite(in_pin1, LOW);
    digitalWrite(in_pin2, HIGH);
  }
  analogWrite(pwm, velocidade);
}

void andar_reto(int vel, int direcao)
{
  // Funcao que coloca a mesma velocidade e direção nos dois motores
  mudar_vel(vel, DIREITO, direcao, in_dir1, in_dir2, pwmA);
  mudar_vel(vel, ESQUERDO, direcao, in_esq1, in_esq2, pwmB);
}

void calibracao()
{
  int milisec_total = 6000; // Tempo total da calibração
  int quant_loop = 5; // Quantas vezes o carro vai para frente e para trás

  for (uint16_t i = 0; i < quant_loop; i++)
  {
    for(uint16_t j = 0; j < (milisec_total/quant_loop); j++){ // 
      sArray.calibrate();
      sLat.calibrate();
      
      if(j < (milisec_total/(quant_loop*2))) // Metade do tempo anda pra frente
        andar_reto(50, FRENTE);
      else // Metade do tempo anda para trás
        andar_reto(50, REH);
      
      delay(20);
    }
  }
}

void ler_sensores()
{

  uint16_t sArraychannels[sArray.getSensorCount()];
  float erro_sensores = sArray.readLineWhite(sArraychannels) - 3500;
  erro_f = -1 * erro_sensores;
 
}

float calcula_PID(float Kp, float Kd)
{
  P = erro_f;
  D = erro_f - erro_anterior;
  erro_anterior = erro_f;
  
  return ((Kp * P) + (Kd * D));
}

float PID_por_trecho(int state){
  float PID_calculado;
  switch(state){
    case RETA:
      PID_calculado = calcula_PID(KpReta, KdReta);
      break;
    case CURVA:
      PID_calculado = calcula_PID(KpCurva, KdCurva);
      break;
    case ZIGZAG:
      PID_calculado = calcula_PID(KpZIG, KdZIG);
      break;
    default:
      PID_calculado = calcula_PID(Kp, Kd);
  }
  return PID_calculado;
}


void controle_motores(float vel, int state)
{
  float PID = PID_por_trecho(state);
  vel_dir = vel + PID;
  vel_esq = vel - PID;

  if (vel_dir < 0)
  {
    vel_dir = -1 * vel_dir;
    if(vel_dir > 255){
      vel_dir = 255;
    }
    mudar_vel(vel_dir, DIREITO, REH, in_dir1, in_dir2, pwmA);
  }
  
  else if (vel_esq < 0)
  {
    if(vel_esq > 255){
      vel_esq = 255;
    }
    vel_esq = -1 * vel_esq;
    mudar_vel(vel_esq, ESQUERDO, REH, in_esq1, in_esq2, pwmB);
  }else{
    if(vel_esq > 255){
      vel_esq = 255;
    }
    if(vel_dir > 255){
      vel_dir = 255;
    }

    mudar_vel(vel_dir, DIREITO, FRENTE, in_dir1, in_dir2, pwmA);
    mudar_vel(vel_esq, ESQUERDO, FRENTE, in_esq1, in_esq2, pwmB);
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

  controle_motores(120, MAPEAMENTO);

}

void controle_com_mapeamento(int encVal){

  if(encVal < 1000){ //início
    controle_motores(145, MAPEAMENTO);
  }
  else if(encVal > 26000 && encVal < 41000){ //zig-zag
    controle_motores(140, ZIGZAG);
  }
  else if(encVal > 41000 && encVal < 41400){ //reta
    controle_motores(235, RETA);
  }
  else if(encVal > 48500){ // parada
    digitalWrite(stby, LOW);
  }
  else{
    controle_motores(167, MAPEAMENTO);
  }

}

int v = 0;
void mapeamento(int encVal){

  timer_in = millis();

  //digitalWrite(led, LOW);
  //digitalWrite(buzzer, LOW);
        
  if(ler_sens_lat() == true){  
    if(timer_in - timer_prev2 >= 15){
      v = (v+1);
      Serial.print(v);
      Serial.print(" ");
      Serial.println(encVal);
          
      //Serial.println(timer_in);

    }

  //digitalWrite(led, HIGH); 
  //digitalWrite(buzzer, HIGH);
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
  //pinMode(led, OUTPUT);
  pinMode(s_lat_esq, INPUT);
  pinMode(s_lat_dir, INPUT);
  //pinMode(buzzer, OUTPUT);

  ESP32Encoder::useInternalWeakPullResistors = UP;

  encoder.attachFullQuad(enc_eq_B, enc_eq_A);
  encoder2.attachFullQuad(enc_dir_A, enc_dir_B);

  digitalWrite(stby, HIGH);

  encoder.clearCount();
  encoder2.clearCount();

  sArray.setTypeMCP3008();
  sArray.setSensorPins((const uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7}, 8, (gpio_num_t)out_s_front, (gpio_num_t)in_s_front, (gpio_num_t)clk, (gpio_num_t)cs_s_front, 1350000, VSPI_HOST);
  sArray.setSamplesPerSensor(5);

  sLat.setTypeAnalogESP();
  sLat.setSensorPins((const uint8_t[]){s_lat_esq, s_lat_dir}, 2);
  sLat.setSamplesPerSensor(5);

  calibracao();

  delay(3000);

  encoder.clearCount();
  encoder2.clearCount();

  //Blynk.run();
  
}

void loop()
{
  timer_in = millis();
     
  if(timer_in - timer_prev >= 15){
    ler_sensores();
    encVal = ((encoder.getCount() + encoder2.getCount())/2);
    mapeamento(encVal);
    controle_sem_mapeamento();
    controle_com_mapeamento(encVal);
      
    timer_prev = timer_in;
  }
}