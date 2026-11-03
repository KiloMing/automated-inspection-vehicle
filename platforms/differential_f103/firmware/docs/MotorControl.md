# 双电机 PID 控制层

MotorControl.c/h 负责两路编码器采样、PID 状态和 PWM 输出；MotorPID.c/h 保留纯算法。
两路使用相同宏参数 KP=8500、KI=2000、KD=0，各自保存积分和历史速度。

初始化顺序：Delay_TickInit()、MotorControl_Init()。后者初始化编码器和电机驱动，并占用 TIM1 进行20ms采样。不要同时启用 MotorScan 或直接控制同一电机。
主循环持续调用 MotorControl_Process()，不要阻塞超过一个控制周期。

```c
MotorControl_SetTarget(1, 150); /* 电机1：150计数/20ms，约0.6m/s */
MotorControl_SetTarget(2, 150); /* 电机2：独立反馈，相同目标 */
MotorControl_SetTarget(1, 0);   /* 立即停止电机1并清空积分 */
MotorControl_Stop();           /* 立即停止两路 */
```

目标单位为有符号编码增量/20ms；内部乘0.004转换为m/s，PWM输出为有符号千分制。
编码器符号必须与对应电机正方向一致；反向指令应在轮子停止后发出。
MotorControl_GetState()读取指定电机的实际编码增量、目标和PWM；该接口及其他非中断接口仅供主循环调用。

TIM1_UP_IRQHandler 调用 MotorControl_SampleIRQ()。采样未及时处理时，两路PWM立即归零，主循环清空目标和PID，需重新设置目标才能继续。
上电目标为0，不自动启动。当前 main 由蓝牙遥控层设置目标并调用 Process，无 VOFA 输出或串口调参流程。未加入标定前馈；遥控配置了静止起步 PWM 下限。
