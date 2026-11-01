#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include <stdint.h>

typedef struct {
    float Pitch;
    float Roll;
    float Yaw;
} Attitude_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float PrevError;
    float Integral;
} PID_t;

void Attitude_Init(void);
void Attitude_Calibrate(void);
void Attitude_Update(float dt);
Attitude_t Attitude_Get(void);

void PID_Init(PID_t *pid, float kp, float ki, float kd);
float PID_Compute(PID_t *pid, float target, float measured, float dt);

#endif
