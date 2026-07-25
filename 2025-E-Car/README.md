# 2025 E 题小车工程

该目录是 2025 年电赛 E 题使用的 MSPM0G3507 小车完整工程，与 `../2025-E-Vision` 中的 MaixCAM 视觉工程配套。

## 主要入口

- `user/main.c`：系统初始化和实时任务入口
- `driver/maixcam.c`：视觉数据帧解析
- `driver/ftServo.c`：云台舵机驱动
- `driver/nuart.c`：UART 中断与 MaixCAM 数据接收
- `apply/developer/subtask.c`：赛题任务流程
- `apply/gray_detection.c`：灰度循迹
- `apply/motor_control.c`：电机控制
- `keil/ncontroller.uvprojx`：Keil 工程文件
- `ticlang/ncontroller.projectspec`：TI Clang / CCS 工程配置

## 编译环境

- MSPM0 SDK `2.00.01.00`
- SysConfig `1.20.0`
- Keil MDK `5.39` + Arm Compiler `6.21`，或 CCS / TI Clang

环境配置的原始说明保存在 `非常重要必读-代码编译环境说明.txt`。

## 视觉联调

MaixCAM UART1 默认使用 `115200` 波特率。MaixCAM A19（TX）连接 MSPM0 PA9（UART1 RX），如需回传则将 MaixCAM A18（RX）连接 MSPM0 PA8（UART1 TX），并确保双方共地。

先在调试页面确认 `get_maixcam_x()` 和 `get_maixcam_y()` 能稳定更新，再启用云台与激光控制。完整数据帧格式见 [../docs/COMMUNICATION.md](../docs/COMMUNICATION.md)。
