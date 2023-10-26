// импорт и объявление библиотек.
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include "MedianFilterLib2.h"
#include "Servo.h"

#include <SPI.h>
#include <SD.h>

Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = 4;


// импорт и объявление датчика расстояния vl53l0x.

#define RIGHT_VL53_ADDRESS 0x38
#define LEFT_VL53_ADDRESS 0x31
#define FORWARD_VL53_ADDRESS 0x32
#define BACKWARD_VL53_ADDRESS 0x33

#define SIDE_RIGHT_VL53_ADDRESS 0x34
#define SIDE_LEFT_VL53_ADDRESS 0x36

#define SHT_VL53_RIGHT 43
#define SHT_VL53_LEFT 53
#define SHT_VL53_FORWARD 51
#define SHT_VL53_BACKWARD 48

#define SHT_VL53_SIDE_RIGHT 41
#define SHT_VL53_SIDE_LEFT 52


VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2; 
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
VL53L0X_RangingMeasurementData_t measure5;
VL53L0X_RangingMeasurementData_t measure6;

Adafruit_VL53L0X right_VL53 = Adafruit_VL53L0X();
Adafruit_VL53L0X left_VL53 = Adafruit_VL53L0X();

Adafruit_VL53L0X forward_VL53 = Adafruit_VL53L0X();
Adafruit_VL53L0X backward_VL53 = Adafruit_VL53L0X();
Adafruit_VL53L0X side_right_VL53 = Adafruit_VL53L0X();
Adafruit_VL53L0X side_left_VL53 = Adafruit_VL53L0X();

// объявление сервомотора и двигателя.

Servo servo; // рулевое управление
Servo motor; // регулятор скорости


// медианные фильтры
MedianFilter2<int> medianFilter_right(4);
MedianFilter2<int> medianFilter_left(4);

// минимальное и максимальное расстояние для использования в функции range_limits

int min_distance = 5;
int max_distance = 100;

// уставка для левого и правого датчиков, расстояние для остановки

int base_distance_left = 50;
int base_distance_right = 50;
int base_distance_stop = 10;

int back_angle = 135;

// базовые значения скоростей

int forward_speed = 1596;
int line_speed = 1580;
int backward_speed = 1345;
int stop_speed = 1495;

unsigned long myTime, last_time; // таймер


// нужные переменные 
float p, i, d, Ei; 
int error, last_error, pid, angle;

// коэффициенты регулятора по умолчанию

// антон => 1610
// float kp_def = 0.12;
// float ki_def = 0.00012;
// float kd_def = 2.5;

// 1596
float kp_def = 0.2;
float ki_def = 0.000015;
float kd_def = 4;


// IR сенсоры
int IR_sensor_1 = 14;
int IR_sensor_2 = 17;
int IR_sensor_3 = 16;
int IR_sensor_4 = 15;


bool IR_sensor_val_prev_1 = false;
bool IR_sensor_val_prev_2 = false;

bool IR_sensor_val_now_1 = false;
bool IR_sensor_val_now_2 = false;

int per_count = 0;