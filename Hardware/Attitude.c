#include "Attitude.h"
#include "MPU6050.h"
#include "Delay.h"
#include <math.h>

#define PI 3.1415926535f
#define RAD_TO_DEG (180.0f / PI)

static Attitude_t currentAttitude = {0.0f, 0.0f, 0.0f};

static float GyroOffsetX = 0.0f;
static float GyroOffsetY = 0.0f;
static float GyroOffsetZ = 0.0f;

static const float Alpha = 0.96f;

void Attitude_Init(void)
{
    currentAttitude.Pitch = 0.0f;
    currentAttitude.Roll = 0.0f;
    currentAttitude.Yaw = 0.0f;
}

void Attitude_Calibrate(void)
{
    int16_t ax, ay, az, gx, gy, gz;
    int32_t sumGx = 0, sumGy = 0, sumGz = 0;
    const int sampleCount = 100;

    for (int i = 0; i < sampleCount; i++)
    {
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        Delay_ms(10);
    }

    GyroOffsetX = (float)sumGx / (float)sampleCount / 16.4f;
    GyroOffsetY = (float)sumGy / (float)sampleCount / 16.4f;
    GyroOffsetZ = (float)sumGz / (float)sampleCount / 16.4f;
}

void Attitude_Update(float dt)
{
    int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
    MPU6050_GetData(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

    float ax = (float)axRaw / 2048.0f;
    float ay = (float)ayRaw / 2048.0f;
    float az = (float)azRaw / 2048.0f;

    float gx = (float)gxRaw / 16.4f - GyroOffsetX;
    float gy = (float)gyRaw / 16.4f - GyroOffsetY;
    float gz = (float)gzRaw / 16.4f - GyroOffsetZ;

    float accRoll  = atan2f(ay, az) * RAD_TO_DEG;
    float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    currentAttitude.Roll  = Alpha * (currentAttitude.Roll + gx * dt) + (1.0f - Alpha) * accRoll;
    currentAttitude.Pitch = Alpha * (currentAttitude.Pitch + gy * dt) + (1.0f - Alpha) * accPitch;

    currentAttitude.Yaw += gz * dt;
    if (currentAttitude.Yaw > 180.0f)  currentAttitude.Yaw -= 360.0f;
    if (currentAttitude.Yaw < -180.0f) currentAttitude.Yaw += 360.0f;
}

Attitude_t Attitude_Get(void)
{
    return currentAttitude;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->PrevError = 0.0f;
    pid->Integral = 0.0f;
}

float PID_Compute(PID_t *pid, float target, float measured, float dt)
{
    float error = target - measured;
    pid->Integral += error * dt;
    if (pid->Integral > 30.0f) pid->Integral = 30.0f;
    if (pid->Integral < -30.0f) pid->Integral = -30.0f;

    float derivative = (error - pid->PrevError) / (dt > 0.0001f ? dt : 0.01f);
    pid->PrevError = error;

    return (pid->Kp * error) + (pid->Ki * pid->Integral) + (pid->Kd * derivative);
}
