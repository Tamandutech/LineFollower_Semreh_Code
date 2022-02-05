


class robo{
    private:
        int speed_left;
        int speed_right;

        int mark_left;
        int mark_right;
        bool sensor_left;
        bool sensor_right;

        int array[8];
        int *array;
        int posicao;

        int encoder_left;
        int encoder_right;

    public:
        int get_speed_left();
        void set_speed_left(int speed_left);
        int get_speed_right();
        void set_speed_right(int speed_right);

        int get_mark_left();
        void set_mark_left(int mark_left);
        int get_mark_right();
        void set_mark_right(int mark_right);
        bool get_sensor_left();
        void  set_sensor_left(int sensor_left);
        bool get_sensor_right();
        void set_sensor_right(int sensor_right);

        int* get_array();
        void set_array(int *array);
        int get_posicao();
        void set_posicao(int posicao);

        int get_encoder_left();
        void set_encoder_left(int encoder_left);
        int get_encoder_right();
        void set_encoder_right(int encoder_right);

};