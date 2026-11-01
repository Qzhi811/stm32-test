#include "stm32f10x.h"
#include "Servo.h"

static float CurrentPitch = 90.0f;
static float CurrentYaw = 90.0f;

static float Clamp(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void Servo_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM2);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_Cmd(TIM2, ENABLE);

    Servo_SetPitch(90.0f);
    Servo_SetYaw(90.0f);
}

void Servo_SetPitch(float Angle)
{
    CurrentPitch = Clamp(Angle, 0.0f, 180.0f);
    uint16_t ccr = (uint16_t)(CurrentPitch / 180.0f * 2000.0f + 500.0f);
    TIM_SetCompare2(TIM2, ccr);
}

void Servo_SetYaw(float Angle)
{
    CurrentYaw = Clamp(Angle, 0.0f, 180.0f);
    uint16_t ccr = (uint16_t)(CurrentYaw / 180.0f * 2000.0f + 500.0f);
    TIM_SetCompare3(TIM2, ccr);
}

void Servo_AddPitch(float Delta)
{
    Servo_SetPitch(CurrentPitch + Delta);
}

void Servo_AddYaw(float Delta)
{
    Servo_SetYaw(CurrentYaw + Delta);
}

float Servo_GetPitch(void)
{
    return CurrentPitch;
}

float Servo_GetYaw(void)
{
    return CurrentYaw;
}
