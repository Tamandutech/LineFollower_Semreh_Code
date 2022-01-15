// portas do esp

//Sensores 
#define numero_sensores 8;

//portas
//array
#define sensores_array 13;  

//laterais
#define sensor_lateral_esquerdo 35;
#define indicador_lateral_esquerdo 1;
#define sensor_lateral_direito 34;
#define indicador_lateral_direito 1;

//drive
#define pwm_direito_a 4;
#define ain1 21;
#define ain2 5;

#define pwm_esquerdo_b 2;
#define b1n1 18;
#define b1n2 19;

//encoder
#define encoder_motor_dir_a 17;
#define encoder_motor_dir_b 16;
#define encoder_motor_esq_a 36;
#define encoder_motor_esq_b 39;


//pista
#define qtde_curvas 1;
#define curvas_esquerda 1; //??????????????
#define distancia da pista 1;

//PID

class pid_config{
    public:
        float Kp = 0;
        float Ki = 0;
        float Kd = 0; //(kp-1)*10

        float P = 0;
        float I = 0; //sempre vai ser 0
        float D = 0; 

        float erro = 0;
        float erro_ant = 0;
        float PID = 0;

    pid_config() {};
};


class CONFIG{
    public:
        pid_config pid;

    CONFIG() {};
};

