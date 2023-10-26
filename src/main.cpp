// импорт файлов
#include "costants.h"
#include "sensors.h"



// ИНИЦИАЛИЗАЦИЯ МОТОРОВ
void init_motors() {
    servo.attach(3);
    motor.attach(2);
    motor.writeMicroseconds(stop_speed);
    servo.write(90);
    delay(5000);
}


// ПИД РЕГУЛЯТОР В УГОЛ СЕРВО
int get_angle(int type, int distance_right, int distance_left, float kp = kp_def, float ki = ki_def, float kd = kd_def) {
    int pid_angle = 0;
    // выбор режима для датчиков
    switch(type) {
    case 0:
        error = distance_right - distance_left;
        break;
    case 1:
        error = distance_right - base_distance_right;
        break;
    case 2:
        error = base_distance_left - distance_left;
        break;
    }

    p = error * kp;
    d = (error - last_error) * kd;
    Ei = Ei + error;
    i = ki * Ei;
    pid = p + i + d;

    last_error = error;
    pid_angle = 90 + pid;

    pid_angle = range_limits(pid_angle, 45, 135);

    

    return pid_angle;
}

// ОТЪЕЗД НАЗАД
void run_backward(int milliseconds) {
    motor.writeMicroseconds(stop_speed);
    delay(100);
    motor.writeMicroseconds(backward_speed);
    delay(milliseconds);
    motor.writeMicroseconds(stop_speed);
    delay(100);
}

// ПРОСТО ЕЗДА ПО КОРИДОРУ
void run_coridor(int type, float kp = kp_def, float ki = ki_def, float kd = kd_def) {
    int distance_forward = forward_VL53.readRange() / 10; // forward
    int distance_right = right_VL53.readRange() / 10; // right
    int distance_left = left_VL53.readRange() / 10; // left

    distance_right = range_limits(distance_right, min_distance, max_distance);
    distance_left = range_limits(distance_left, min_distance, max_distance);
    distance_forward = range_limits(distance_forward, min_distance, max_distance);

    medianFilter_right.AddValue(distance_right);
    medianFilter_left.AddValue(distance_left);

    Serial.print(medianFilter_right.GetFiltered());
    Serial.print("  ");
    Serial.println(distance_right);   

    angle = get_angle(type, medianFilter_right.GetFiltered(), medianFilter_left.GetFiltered(), kp, ki, kd);


    // ОТЪЕЗД НАЗАД - ЕСЛИ НАДО РАССКОМЕНТИРОВАТЬ НО ЛУЧШЕ НЕ НАДО ИЛИ РЕГУЛИРОВАТЬ СТОП РАССТОЯНИЕ

    // if (distance_forward < base_distance_stop) {
    //     delay(100);
    //     motor.writeMicroseconds(stop_speed);
    //     delay(100);
    //     servo.write(back_angle);
    //     run_backward(1000);
    // } 
    // else {
    //     // motor.writeMicroseconds(forward_speed - int(abs(90-angle) / 7));
    //     motor.writeMicroseconds(forward_speed);
    //     servo.write(angle);
    // }

    motor.writeMicroseconds(forward_speed);
    servo.write(angle);
    
}

// ПРОЕЗД ПО КОРИДОРУ ПО ВРЕМЕНИ
void run_time_coridor(unsigned long run_time, int type, float kp = kp_def, float ki = ki_def, float kd = kd_def) {
    myTime = millis();
    for (;;) {
        if (myTime + run_time >= millis()){
            run_coridor(type, kp, ki, kd);
        } else {
            motor.writeMicroseconds(stop_speed);
            delay(500);
            break;
        }
    }
}


// ПРОСТО ЛИНИЯ
void run_line () {
    IR_sensor_val_now_1 = digitalRead(IR_sensor_1);
    IR_sensor_val_now_2 = digitalRead(IR_sensor_2);

    motor.writeMicroseconds(line_speed);

    // УГОЛ ПОВОРОТА КОЛЕСА ИЛИ 45 ИЛИ 135 - МОЖНО ПОСТАВИТЬ ПОМЕНЬШЕ

    if (IR_sensor_val_now_1) {
        servo.write(45);
    }
    else if (IR_sensor_val_now_2) {
        servo.write(135);
    }
    else{ 
        servo.write(90);
    }
    
    // ЗАДЕРЖКА МЕЖДУ ПОВОРОТАМИ КОЛЕС
    delay(15);
}


// ЛИНИЯ ПО ВРЕМЕНИ
void run_line_time (int run_time) {
    myTime = millis();
    for (;;) {
        if (myTime + run_time >= millis()){
            run_line();
        } else {
            motor.writeMicroseconds(stop_speed);
            delay(500);
            break;
        }
    }
}


// ПРОЕЗД ПО КОЛ-ВУ ПЕРЕКРЕСТКОВ
void run_count_per (int count, int type) {
    for(;;) {
        if (per_count < count) {
            run_coridor(type);
        }
        else{ 
            break;
        }
        counter_per();

        // ЗАДЕРЖКА МЕЖДУ СЧИТЫВАНИЯМИ
        delay(15);

    }
    
    motor.writeMicroseconds(stop_speed);

    per_count = 0;
}


// инициализация сд карты
void initSD () {
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        return;
    }
}

// инициализация всего и вся
void init_perepherial() {
  
    pinMode(9, INPUT_PULLUP);

    init_sensors();
    init_motors();

    // for(;;) {
    // if(!digitalRead(41) == 1){
    //     break;
    // } else {}
    // }

    motor.writeMicroseconds(stop_speed);
    delay(500);
} 


void setup() {

    // Инициализируем основные службы
    Serial.begin(9600);
    init_perepherial();


    // ПРИМЕР ПРОГРАММЫ И ПРИМЕНЕНИЯ ФУНКЦИЙ

    // run_count_per(4, 0);
    // run_time_coridor(200, 0);
    // run_count_per(1, 0);
    // run_time_coridor(1000, 2, ki_def=0.01, kd_def=7);
    // run_line_time(4000);



    // ЗДЕСЬ ПИСАТЬ НОВУЮ













}

void loop() {
    // ЗНАЧЕНИЯ ДАТЧИКОВ
    // read_three_sensors();
    
    // КОРИДОР БЕСКОНЕЧНЫЙ
    // run_coridor(0);

    // ЛИНИЯ БЕСКОНЕЧНАЯ
    // run_line();
}