# 作业

## CAN_cmd_chassis.txt

电控组最后一次作业：底盘 CAN 发送数据函数。

- 要求：类比云台 `gimbal` 发送函数，编写底盘 4 个电机的发送函数
- 帧格式：标准帧，`DLC = 8`（4 个电机 × 2 字节）
- 内容：所有用到的变量单独定义，函数与变量逐行注释

函数原型：

```c
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
```

文件：[CAN_cmd_chassis.txt](./CAN_cmd_chassis.txt)
