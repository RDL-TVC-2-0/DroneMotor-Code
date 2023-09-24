#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>

//#include <Servo.h>
#include "C:\Program Files (x86)\Arduino\libraries\Servo"
//#include "C:\Users\derek\AppData\Local\Arduino15\packages\teensy\hardware\avr\1.58.1\libraries\Servo\Servo.h"

File BenchData;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

#define SERVOA_PIN 2
#define SERVOB_PIN 3
Servo servoA;
Servo servoB;

void setup() {

}

void loop() {

}