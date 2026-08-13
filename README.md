# NUEDC Car & Vision

全国大学生电子设计竞赛小车与视觉开源工程，包含 2024 年 H 题小车代码，以及相互配套的 2025 年 E 题 MaixCAM 视觉代码和 MSPM0G3507 小车代码。

## 项目内容

| 模块 | 平台 | 主要内容 | 入口 |
| --- | --- | --- | --- |
| `2024-H-Car` | TI MSPM0G3507 | 2024 年 H 题小车控制、灰度循迹、电机与编码器、IMU、OLED 等 | `user/main.c` |
| `2025-E-Vision` | MaixCAM / MaixPy | 2025 年 E 题目标框识别、透视变换、圆心定位和串口输出 | `app.yaml` -> `ff.py` |
| `2025-E-Car` | TI MSPM0G3507 | 2025 年 E 题配套小车、循迹、MaixCAM 通信、云台与激光控制 | `user/main.c` |

三个目录均完整保留原工程结构。两个 MSPM0 工程分别携带所需 SDK 源码和工程配置，避免重组目录后出现相对路径失效。

## 硬件与软件

### 小车端

- 主控：TI MSPM0G3507
- MSPM0 SDK：`2.00.01.00`
- SysConfig：`1.20.0`
- Keil MDK：`5.39`，Arm Compiler：`6.21`
- 下载器：DAPLink、J-Link 或 XDS110，不支持 ST-Link
- 2025 E 题工程另提供 TI Clang / CCS 工程配置

### 视觉端

- MaixCAM 或 MaixCAM-Pro，默认 GC4653 摄像头
- MaixPy 系统镜像：已在 `v4.11.8` 测试
- MaixVision
- OpenCV、NumPy 和 MaixPy 内置接口
- [黑框检测模型](https://maixhub.com/model/zoo/1159)

## 快速开始

### 2024 H 题小车

1. 安装上面列出的 Keil、Arm Compiler、MSPM0 SDK 和 SysConfig。
2. 打开 `2024-H-Car/keil/ncontroller.uvprojx`。
3. 检查 `ncontroller.syscfg`、下载器和目标芯片配置。
4. 根据实际接线完成传感器标定、PID 调参后编译下载。

### 2025 E 题视觉

1. 下载检测模型，将文件放到 MaixCAM 的 `/root/models`。
2. 确认 `2025-E-Vision/ff.py` 中的 `model_path` 指向实际 `.mud` 文件。
3. 使用 MaixVision 打开 `2025-E-Vision`，运行 `app.yaml` 指定的 `ff.py`。
4. 根据镜头、距离和光照调整分辨率、裁切、白平衡和识别参数。

### 2025 E 题小车

1. Keil 用户打开 `2025-E-Car/keil/ncontroller.uvprojx`。
2. CCS 用户可导入 `2025-E-Car/ticlang/ncontroller.projectspec`。
3. 将 MaixCAM UART1 与 MSPM0 UART1 交叉连接并共地，默认波特率为 `115200`。
4. 先验证视觉坐标数据，再分别调试循迹、云台和激光控制。

视觉与小车的通信帧定义见 [docs/COMMUNICATION.md](docs/COMMUNICATION.md)。更详细的模块说明见各目录 README。

## 目录说明

```text
NUEEDC-Car-Vision/
├── 2024-H-Car/       # 2024 H 题完整小车工程
├── 2025-E-Vision/    # 2025 E 题 MaixCAM 工程
├── 2025-E-Car/       # 2025 E 题完整小车工程
├── docs/             # 通信协议等补充文档
├── CONTRIBUTING.md
├── LICENSE
└── NOTICE
```

`apply/`、`driver/`、`user/` 是主要业务源码目录，`keil/` 和 `ticlang/` 保存工程配置，`source/` 保存 MSPM0 SDK 与第三方依赖。`.metadata/`、Keil `Objects/` 和 `Listings/` 等缓存或编译输出不会加入 Git。

## 使用提示

- 竞赛代码与具体机械结构、接线、镜头和场地强相关，不能保证下载后无需调参即可复现成绩。
- 上电前核对电机方向、舵机行程、激光安全、电源电压和所有引脚定义。
- 建议按“单模块测试 -> 串口联调 -> 整车低速测试 -> 完整任务”的顺序调试。
- C 工程中部分历史注释的文本编码可能依赖原开发环境；如显示异常，请在编辑器中尝试 GBK/UTF-8 编码，避免直接批量转换源码。

## 开源说明

本人曾获 2024 年省赛一等奖和 2025 年国赛一等奖。整理并开源这些工程，是希望大家减少重复搭建和查找资料的时间，把更多精力投入方案设计、系统调试和比赛本身。

祝大家比赛顺利，取得理想成绩！

项目原创部分采用 [MIT License](LICENSE)。TI MSPM0 SDK、CMSIS、MaixPy 相关代码及其他第三方组件仍遵循各自文件中的版权和许可声明，详情见 [NOTICE](NOTICE)。
