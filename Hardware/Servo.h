#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

void Servo_Init(void);
void Servo_SetPitch(float Angle);
void Servo_SetYaw(float Angle);
void Servo_AddPitch(float Delta);
void Servo_AddYaw(float Delta);
float Servo_GetPitch(void);
float Servo_GetYaw(void);

#endif
