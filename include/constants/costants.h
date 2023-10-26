// импорт и объявление библиотек.
#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
#include "Servo.h"

#include <SPI.h>
#include <SD.h>


Sd2Card card;
SdVolume volume;
SdFile root;

const int chipSelect = 4;



// импорт и объявление датчика расстояния vl53l0x.

#define RIGHT_VL53_ADDRESS 0x30
#define LEFT_VL53_ADDRESS 0x31
#define FORWARD_VL53_ADDRESS 0x32
#define BACKWARD_VL53_ADDRESS 0x33

#define SIDE_RIGHT_VL53_ADDRESS 0x34
#define SIDE_LEFT_VL53_ADDRESS 0x35

#define SHT_VL53_RIGHT 52
#define SHT_VL53_LEFT 53
#define SHT_VL53_FORWARD 51
#define SHT_VL53_BACKWARD 48

#define SHT_VL53_SIDE_RIGHT 47
#define SHT_VL53_SIDE_LEFT 46

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

// минимальное и максимальное расстояние для использования в функции range_limits

int min_distance = 5;
int max_distance = 80;

// уставка для левого и правого датчиков, расстояние для остановки

int base_distance_left = 70;
int base_distance_right = 70;
int base_distance_stop = 50;

// базовые значения скоростей

int forward_speed = 1570;
int backward_speed = 1345;
int stop_speed = 1500;

unsigned long myTime, last_time; // таймер


// нужные переменные 
float p, i, d, Ei; 
int error, last_error, pid, angle;

// коэффициенты регулятора по умолчанию
float kp_def = 0.56;
float ki_def = 0.00006;
float kd_def = 5;

