# 自研四轴/穿越机嵌入式飞控全栈项目 (DIY FPV Flight Controller)

本项目是一个从零自研的嵌入式飞行控制器全栈工程，包含 **自主 PCB 硬件设计（STM32F405 + ICM-42688-P）** 与 **全套自研飞控固件（驱动、数字滤波、Mahony 姿态解算、双环串级 PID、CRSF 遥控解析与 DShot 数字电调驱动）**。

---

## 一、 项目目录结构索引

```
agent0/
├── README.md                              # 项目总览与使用手册
├── docs/                                  # 核心技术文档与设计规范
│   ├── BOM_and_Component_Selection.md     # 板载元器件选型与外购动力清单 (BOM)
│   ├── Hardware_Design_and_Pinout.md      # STM32F405 引脚分配表与 PCB 4层板布线规范
│   └── Control_Theory_and_Algorithms.md   # 飞控数学原理推导 (Mahony / 串级PID / DShot)
├── hardware/                              # PCB 设计工程文件与规范 (Altium Designer)
│   ├── README.md                          # AD 原理图与投板检查清单
│   └── Altium_Designer_Guide.md           # AD 叠层管理(4层板)、设计规则与Gerber导出全指南
├── firmware/                              # 自研嵌入式飞控固件源码
│   ├── include/
│   │   └── config.h                       # 全局系统参数、循环频率与安全限幅
│   ├── Middlewares/
│   │   ├── filter.h / .c                  # 一阶低通与双二阶陷波滤波器 (滤除电机高频震动)
│   │   ├── ahrs_mahony.h / .c             # Mahony 姿态解算 (四元数微分与欧拉角转换)
│   │   ├── pid.h / .c                     # 工业级双环 PID 控制器 (抗饱和+微分先行)
│   │   └── mixer.h / .c                   # X型四轴混控器 (推力与力矩分配)
│   ├── Devices/
│   │   ├── dev_icm42688.h / .c            # ICM-42688-P 6轴 IMU 寄存器配置与高速SPI驱动
│   │   └── dev_crsf.h / .c                # ELRS/CRSF 16通道遥控协议解析与CRC校验
│   ├── Drivers/
│   │   └── drv_dshot.h / .c               # DShot300/600 数字电调协议组帧与DMA编码
│   └── App/
│       ├── failsafe.h / .c                # 解锁安全检查与失控保护状态机
│       ├── scheduler.h                    # 毫秒级时间片轮询调度器 (1kHz/100Hz/10Hz)
│       └── main.c                         # 飞控主程序生命周期与任务集成入口
└── tools/
    └── vofa_protocol.md                   # VOFA+ 上位机实时波形与 3D 姿态调试配置
```

---

## 二、 关键开发阶段与实操指南

### 阶段一：硬件原理图与 PCB 制版（当前即可开始）
1. 参考 [`docs/Hardware_Design_and_Pinout.md`](docs/Hardware_Design_and_Pinout.md) 查看 STM32F405 推荐的引脚复用定义。
2. 打开 **立创EDA 专业版** 或 **KiCad**，按照 [`hardware/README.md`](hardware/README.md) 中的模块清单画出原理图。
3. 参考 [`docs/BOM_and_Component_Selection.md`](docs/BOM_and_Component_Selection.md) 导出物料清单并在立创商城/淘宝采购打样。

### 阶段二：硬件冷测与通电调试（Bring-up）
1. **防短路**：使用万用表蜂鸣档测试 3.3V 与 5V 供电轨对地电阻，严禁带短路上电。
2. **连接调试器**：用 DAP-Link 或 ST-Link 连接 SWD 调试引脚，识别芯片内核。
3. **SPI 通信点亮**：调用 `ICM42688_Init()` 读取芯片 ID（`0x47`），确认焊接成功。

### 阶段三：算法验证与上位机联调
1. 将飞控板通过 USB 虚拟串口或 USB-TTL 模块连接电脑。
2. 按照 [`tools/vofa_protocol.md`](tools/vofa_protocol.md) 打开 VOFA+ 上位机。
3. 旋转自制飞控板，观察上位机中的 3D 姿态模型跟踪是否灵敏无漂移。

### 阶段四：动力联调与首飞（⚠️ 安全第一）
> [!CAUTION]
> **室内调参、写代码、烧录固件时，绝对不可安装螺旋桨！**
> 只有在空旷室外进行首飞拉升测试时，方可上桨。
