#ifndef PIDcontrol_h
#define PIDcontrol_h

// PID Control Parameters for Pitch
#define kp_pitch 0.0
#define ki_pitch 0.0
#define kd_pitch 0.0

// PID Control Parameters for Yaw
#define kp_yaw 0.0
#define ki_yaw 0.0
#define kd_yaw 0.0

// Target Pitch and Yaw Angles
float target_pitch = 0.0;
float target_yaw = 0.0;

// Initializing Integral Error
float int_pitch = 0.0;
float int_yaw = 0.0;

void PIDcontrol(float orient[6], float dt, float& outputPitch, float& outputYaw);

#endif