#include "stm32f10x.h"

/**
  * ===================================================================
  * 电控作业：4个LED流水灯 + 按键切换流向（进阶任务）
  * 硬件引脚分配：
  *   LED1: PA0 (低电平点亮)
  *   LED2: PA1 (低电平点亮)
  *   LED3: PA2 (低电平点亮)
  *   LED4: PA3 (低电平点亮)
  *   KEY1: PB1 (上拉输入，按键另一端接GND，按下为低电平)
  * ===================================================================
  */

/* 微秒延时函数 (基于 SysTick 滴答定时器) */
void Delay_us(uint32_t xus)
{
    SysTick->LOAD = 72 * xus;               // 72MHz系统时钟，计数72次为1微秒
    SysTick->VAL = 0x00;                    // 清空当前计数值
    SysTick->CTRL = 0x00000005;             // 启动定时器 (CLKSOURCE=HCLK, ENABLE=1)
    while (!(SysTick->CTRL & 0x00010000));  // 等待 COUNTFLAG 置位
    SysTick->CTRL = 0x00000004;             // 关闭定时器
}

/* 毫秒延时函数 */
void Delay_ms(uint32_t xms)
{
    while (xms--)
    {
        Delay_us(1000);
    }
}

/* 4个LED引脚数组 */
const uint16_t LED_PINS[4] = {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3};

/* 硬件 GPIO 初始化 */
void Hardware_Init(void)
{
    // 1. 开启 GPIOA 和 GPIOB 的外设时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // 2. 初始化 PA0~PA3 为推挽输出 (用于驱动 4 个 LED)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 默认输出高电平，4个LED全灭 (因为是低电平驱动)
    GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);

    // 3. 初始化 PB1 为上拉输入 (用于检测按键)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 内部上拉，平时为高电平
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * 按键检测函数 (带硬件消抖与松手检测)
  * 返回值：1 表示有效按下，0 表示无动作
  */
uint8_t Key_GetPressed(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0) // 检测到按下接通GND
    {
        Delay_ms(20); // 前沿消抖
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
        {
            while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0); // 等待按键松手
            Delay_ms(20); // 后沿消抖
            return 1;
        }
    }
    return 0;
}

int main(void)
{
    Hardware_Init(); // 初始化硬件

    int8_t current_led = 0; // 当前亮灯索引 (0~3)
    uint8_t direction = 0;  // 0: 正向流动 (LED1->LED4), 1: 反向流动 (LED4->LED1)

    while (1)
    {
        // 1. 先熄灭所有 LED
        GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);

        // 2. 点亮当前序号的 LED (输出低电平)
        GPIO_ResetBits(GPIOA, LED_PINS[current_led]);

        // 3. 在 200ms 的亮灯延时期间进行高频按键轮询检测
        //    (每 10ms 检测一次，保证按键随按随响应，绝不丢键)
        for (int i = 0; i < 20; i++)
        {
            Delay_ms(10);
            if (Key_GetPressed())
            {
                direction = !direction; // 按键触发：立即反转流水灯流向
                break;                  // 立即跳出当前延时，转向下一个灯
            }
        }

        // 4. 根据当前方向，计算下一个该点亮的 LED
        if (direction == 0)
        {
            current_led++;
            if (current_led > 3)
            {
                current_led = 0; // 0 -> 1 -> 2 -> 3 -> 0
            }
        }
        else
        {
            current_led--;
            if (current_led < 0)
            {
                current_led = 3; // 3 -> 2 -> 1 -> 0 -> 3
            }
        }
    }
}
