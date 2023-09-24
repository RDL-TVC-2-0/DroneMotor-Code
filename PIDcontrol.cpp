#include "PIDcontrol.h"

void PIDcontrol(double orient[6], double dt, double& outputPitch, double& outputYaw) {
	// Assign orientation values to pitch, yaw, pitch_rate, and yaw_rate variables
	double pitch = orient[1];
	double yaw = orient[2];
	double pitch_rate = orient[4];
	double yaw_rate = orient[5];


	// Error calculation
	double pitch_error = target_pitch - pitch;
	double yaw_error = target_yaw - yaw;

	// Proportional error
	double prop_pitch = kp_pitch * pitch_error;
	double prop_yaw = kp_yaw * yaw_error;

	// Integral error
	int_pitch += ki_pitch * pitch_error * dt;
	int_yaw += ki_yaw * yaw_error * dt;

	// Derivative error
	double der_pitch = kd_pitch * pitch_rate;
	double der_yaw = kd_yaw * yaw_rate;

	// PID Output
	outputPitch = prop_pitch + int_pitch - der_pitch;
	outputYaw = prop_yaw + int_yaw - der_yaw;
}