# 2024 H 题小车工程

该目录是 2024 年电赛 H 题使用的 MSPM0G3507 小车完整工程，保留 Keil 工程、SysConfig 配置、业务源码、驱动和随工程使用的 MSPM0 SDK。

## 主要入口

- `user/main.c`：系统初始化和周期任务入口
- `apply/developer/subtask.c`：任务流程与赛题逻辑
- `apply/gray_detection.c`：灰度检测与循迹处理
- `apply/motor_control.c`：电机闭环控制
- `apply/pid.c`：PID 控制器
- `driver/`：编码器、IMU、OLED、串口、Flash 等底层驱动
- `ncontroller.syscfg`：MSPM0 外设配置
- `keil/ncontroller.uvprojx`：Keil 工程文件

## 编译环境

- MSPM0 SDK `2.00.01.00`
- SysConfig `1.20.0`
- Keil MDK `5.39`
- Arm Compiler `6.21`

使用其他版本时可能需要重新选择工具链、修正 SDK 路径或暂时将优化等级设为 `-O0`。下载器可使用 DAPLink、J-Link 或 XDS110，MSPM0 不支持 ST-Link。

## 调试建议

首次运行前检查 `user/main.c` 顶部的引脚分配，以及灰度传感器、电机方向、编码器、IMU 和舵机的实际接线。建议先架空车轮进行电机闭环测试，再低速调试循迹参数。
