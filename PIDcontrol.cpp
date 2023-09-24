#include "PIDcontrol.h"

void PIDcontrol(float orient[6], float dt, float& outputPitch, float& outputYaw) {
	// Assign orientation values to pitch, yaw, pitch_rate, and yaw_rate variables
	float pitch = orient[1];
	float yaw = orient[2];
	float pitch_rate = orient[4];
	float yaw_rate = orient[5];


	// Error calculation
	float pitch_error = target_pitch - pitch;
	float yaw_error = target_yaw - yaw;

	// Proportional error
	float prop_pitch = kp_pitch*pitch_error;
	float prop_yaw = kp_yaw*yaw_error;

	// Integral error
	int_pitch += ki_pitch*pitch_error*dt;
	int_yaw += ki_yaw*yaw_error*dt;

	// Derivative error
	float der_pitch = kd_pitch*pitch_rate;
	float der_yaw = kd_yaw*yaw_rate;

	// PID Output
	outputPitch = prop_pitch + int_pitch - der_pitch;
	outputYaw = prop_yaw + int_yaw - der_yaw;
}