#define DEBUG true  // set to true for debug output, false for no debug ouput
//#define Serial if(DEBUG)Serial

#include <Wire.h>

#include <Servo.h>
#include <SD.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "PIDcontrol.h"


File DataFile;

#define BNO055_SAMPLERATE_DELAY_MS 10
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define SERVO_PIN_PITCH 2
#define SERVO_PIN_YAW 3
Servo servoPitch;
Servo servoYaw;

unsigned long oldTime;

void setup() {
	Serial.begin(115200);
	while (DEBUG && !Serial) delay(100); // wait for serial connection
	if (DEBUG) delay(3000);
	Serial.println("CODE STARTED");
	

	// SD card init
	if (!SD.begin(BUILTIN_SDCARD)) {
		Serial.println("SD Card failed or not present");
		while(true) delay(100);
	}
	Serial.println("SD Card initialized");
	DataFile = SD.open("flightLog.dat", FILE_WRITE_BEGIN);

	// BNO055 setup
	if (!bno.begin()) {
		Serial.println("BNO055 failed");
		while(true) delay(100);
	}
	Serial.println("BNO055 initialized");
	bno.setExtCrystalUse(true);

	Serial.println("Calibrating BNO055");
	uint8_t cal, gyro, accel, mag = 0;
	bno.getCalibration(&cal, &gyro, &accel, &mag);
	Serial.print(cal);
	Serial.print(", ");
	Serial.print(gyro);
	Serial.print(", ");
	Serial.print(accel);
	Serial.print(", ");
	Serial.println(mag);

	// Servo setup
	servoPitch.attach(SERVO_PIN_PITCH);
	servoYaw.attach(SERVO_PIN_YAW);

	servoPitch.write(0);
	servoYaw.write(0);

	delay(1000);
	oldTime = micros();
}

void loop() {
	unsigned long sampleTimer = micros();

	// Possible vector values can be:
	// - VECTOR_ACCELEROMETER - m/s^2
	// - VECTOR_MAGNETOMETER  - uT
	// - VECTOR_GYROSCOPE     - rad/s
	// - VECTOR_EULER         - degrees
	// - VECTOR_LINEARACCEL   - m/s^2
	// - VECTOR_GRAVITY       - m/s^2

	//sensors_event_t orientationData;
	//bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

	imu::Vector<3> orientationData;
	orientationData = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // degrees
	//orientationData.x();
    //orientationData.y();
    //orientationData.z();

	imu::Vector<3> gyroscopeData;
	gyroscopeData = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // rad/s
	//gyroscopeData.x() * SENSORS_DPS_TO_RADS;
    //gyroscopeData.y() * SENSORS_DPS_TO_RADS;
    //gyroscopeData.z() * SENSORS_DPS_TO_RADS;

	//imu::Vector<3> linearAccelData;
	//linearAccelData = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // m/s^2
    //linearAccelData.x();
    //linearAccelData.y();
    //linearAccelData.z();


	double orient[6];
	orient[0] = orientationData.x();
	orient[1] = orientationData.y();
	orient[2] = orientationData.z();
	orient[3] = gyroscopeData.x();
	orient[4] = gyroscopeData.y();
	orient[5] = gyroscopeData.z();


	double outputPitch, outputYaw;
	double deltaTime = (micros() - oldTime) / 1000000.0;
    oldTime = micros();
	PIDcontrol(orient, deltaTime, outputPitch, outputYaw);


	servoPitch.write(outputPitch);
	servoYaw.write(outputYaw);


	Serial.print("DeltaT:");
	Serial.print(deltaTime);
	Serial.print(",");
	Serial.print("Pitch:");
	Serial.print(orientationData.x());
	Serial.print(",");
	Serial.print("Yaw:");
	Serial.print(orientationData.y());
	Serial.print(",");
	Serial.print("Roll:");
	Serial.print(orientationData.z());

	Serial.print("Pitch*:");
	Serial.print(gyroscopeData.x());
	Serial.print(",");
	Serial.print("Yaw*:");
	Serial.print(gyroscopeData.y());
	Serial.print(",");
	Serial.print("Roll*:");
	Serial.print(gyroscopeData.z());

	Serial.print(",");
	Serial.print("OutputPitch:");
	Serial.print(outputPitch);
	Serial.print(",");
	Serial.print("OutputYaw:");
	Serial.print(outputYaw);


	Serial.println("\n");
	while ((micros() - sampleTimer) < (BNO055_SAMPLERATE_DELAY_MS * 1000));
}
