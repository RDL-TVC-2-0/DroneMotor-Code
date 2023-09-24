#ifndef GetOrientation_h
#define GetOrientation_h

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

void orientation(Adafruit_BNO055& bno, float orient[6]);

void quaternionToEuler(float qx, float qy, float qz, float qw, float* pitch, float* yaw, float* roll);

imu::Quaternion quaternionConjugate(imu::Quaternion quat);

void plotOrientation(float pitch, float yaw, float roll);

#endif