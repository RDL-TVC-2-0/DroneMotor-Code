#ifndef PIDcontrol_h
#define PIDcontrol_h

// PID Control Parameters for Pitch
#define kp_pitch 0
#define ki_pitch 0
#define kd_pitch 0

// PID Control Parameters for Yaw
#define kp_yaw 0
#define ki_yaw 0
#define kd_yaw 0

void PIDcontrol(double orient[6], double dt, double& outputPitch, double& outputYaw);

#endif