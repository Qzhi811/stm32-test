#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Servo.h"
#include "Serial.h"
#include "Attitude.h"
#include <stdio.h>

static uint8_t ClosedLoopMode = 0;
static PID_t pidPitch;
static PID_t pidYaw;

int main(void)
{
    OLED_Init();
    MPU6050_Init();
    Servo_Init();
    Serial_Init();
    Attitude_Init();

    PID_Init(&pidPitch, 0.8f, 0.0f, 0.05f);
    PID_Init(&pidYaw,   0.8f, 0.0f, 0.05f);

    OLED_ShowString(1, 1, "RM Gimbal Init ");
    OLED_ShowString(2, 1, "Calibrating... ");
    OLED_ShowString(3, 1, "Keep Still!    ");
    Serial_SendString("\r\n========================================\r\n");
    Serial_SendString("  RoboMaster Course Design: 2-DOF Gimbal\r\n");
    Serial_SendString("  Calibrating Gyro Zero Bias...         \r\n");
    Serial_SendString("========================================\r\n");

    Attitude_Calibrate();

    OLED_Clear();
    Serial_SendString("Calibration Done! System Running.\r\n");
    Serial_SendString("Commands: W/S (Pitch +/-5) | A/D (Yaw +/-5) | R (Reset) | C (Toggle PID)\r\n\r\n");

    uint32_t loopCount = 0;
    char lineBuffer[17];

    while (1)
    {
        Attitude_Update(0.01f);
        Attitude_t att = Attitude_Get();

        if (Serial_GetRxFlag())
        {
            uint8_t cmd = Serial_GetRxData();
            if (cmd == 'w' || cmd == 'W')
            {
                Servo_AddPitch(+5.0f);
                printf(">> CMD [W]: Pitch +5 -> %.1f deg\r\n", Servo_GetPitch());
            }
            else if (cmd == 's' || cmd == 'S')
            {
                Servo_AddPitch(-5.0f);
                printf(">> CMD [S]: Pitch -5 -> %.1f deg\r\n", Servo_GetPitch());
            }
            else if (cmd == 'a' || cmd == 'A')
            {
                Servo_AddYaw(-5.0f);
                printf(">> CMD [A]: Yaw -5 -> %.1f deg\r\n", Servo_GetYaw());
            }
            else if (cmd == 'd' || cmd == 'D')
            {
                Servo_AddYaw(+5.0f);
                printf(">> CMD [D]: Yaw +5 -> %.1f deg\r\n", Servo_GetYaw());
            }
            else if (cmd == 'r' || cmd == 'R')
            {
                Servo_SetPitch(90.0f);
                Servo_SetYaw(90.0f);
                printf(">> CMD [R]: Reset to 90 deg\r\n");
            }
            else if (cmd == 'c' || cmd == 'C')
            {
                ClosedLoopMode = !ClosedLoopMode;
                if (!ClosedLoopMode)
                {
                    Servo_SetPitch(90.0f);
                    Servo_SetYaw(90.0f);
                }
                printf(">> CMD [C]: Closed-loop Mode = %s\r\n", ClosedLoopMode ? "ENABLED (PID)" : "DISABLED (MANUAL)");
            }
        }

        if (ClosedLoopMode)
        {
            float pitchCorrection = PID_Compute(&pidPitch, 0.0f, att.Pitch, 0.01f);
            float rollCorrection  = PID_Compute(&pidYaw,   0.0f, att.Roll,  0.01f);
            Servo_SetPitch(90.0f - pitchCorrection);
            Servo_SetYaw(90.0f - rollCorrection);
        }

        loopCount++;
        if (loopCount >= 10)
        {
            loopCount = 0;

            sprintf(lineBuffer, "P:%+4d  R:%+4d ", (int)att.Pitch, (int)att.Roll);
            OLED_ShowString(1, 1, lineBuffer);

            sprintf(lineBuffer, "Y:%+4d [%s]   ", (int)att.Yaw, ClosedLoopMode ? "PID" : "MAN");
            OLED_ShowString(2, 1, lineBuffer);

            sprintf(lineBuffer, "Pitch_S: %3d deg", (int)Servo_GetPitch());
            OLED_ShowString(3, 1, lineBuffer);

            sprintf(lineBuffer, "Yaw_S:   %3d deg", (int)Servo_GetYaw());
            OLED_ShowString(4, 1, lineBuffer);

            printf("Pitch:%+6.1f, Roll:%+6.1f, Yaw:%+6.1f | S_P:%5.1f, S_Y:%5.1f | [%s]\r\n",
                   att.Pitch, att.Roll, att.Yaw,
                   Servo_GetPitch(), Servo_GetYaw(),
                   ClosedLoopMode ? "PID_STAB" : "MANUAL");
        }

        Delay_ms(10);
    }
}
