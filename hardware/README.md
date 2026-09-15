# 自研飞控 PCB 工程设计指导 (Hardware - Altium Designer)

本目录用于存放你的自研飞控原理图与 PCB 设计工程源文件，项目全面采用 **Altium Designer (AD)** 作为核心 EDA 制版工具。

> [!TIP]
> 详细的 AD 工程模板、4层板叠层设置（Layer Stack Manager）、设计规则配置（PCB Rules）与投板 Gerber 导出指南，请参阅专门文档：
> 📖 **[Altium_Designer_Guide.md](Altium_Designer_Guide.md)**

---

## 一、 推荐设计规格 (Altium Designer)

* **板框尺寸**：`20.0mm × 20.0mm`（正方形），圆角半径 `R = 1.0mm ~ 1.2mm`。
* **安装孔位**：四角各设 1 个 `M2` 规格通孔（孔径建议 `2.1mm`，支持加装硅胶减震球），四孔中心距为 `20.0mm × 20.0mm`。
* **板层设置**：嘉立创等标准高可靠 **四层板 (4-Layer PCB)**：
  * **Top Layer**: 顶层高频信号走线 + 核心元器件 (MCU、IMU、LDO)
  * **Mid-Layer 1 (GND Plane)**: 完整地平面（不走信号线，提供最低阻抗回流路径）
  * **Mid-Layer 2 (PWR Plane)**: 电源层（3.3V 洁净铺铜与 5V 主供电总线）
  * **Bottom Layer**: 底层信号走线 + 大功率电调接线焊盘

---

## 二、 原理图设计步骤清单

按照以下模块在立创EDA中分块绘制原理图：

- [ ] **1. 电源与滤波模块 (Power)**：
  - 输入 5V 电源滤波（10μF 陶瓷电容）
  - ME6211C33 / RT9193-33 LDO 输出 3.3V
  - 磁珠隔离形成 `3V3_IMU` 洁净电源轨
- [ ] **2. MCU 最小系统 (STM32F405RGT6)**：
  - VDD/VSS 每个电源对就近放置 0.1μF 去耦电容
  - HSE 8MHz 外部无源晶振 + 2×18pF 负载电容
  - NRST 硬件复位按键 + 100nF 电容
  - BOOT0 下拉 10k 电阻 + 轻触按键（上电拉高进入 DFU）
- [ ] **3. IMU 惯性传感器 (ICM-42688-P)**：
  - SPI1 接口 (SCK: PA5, MISO: PA6, MOSI: PA7)
  - 片选 CS: PC4，数据就绪中断 INT: PC5
  - VDD / VDDIO 各配置 0.1μF + 2.2μF 贴片电容
- [ ] **4. 接口与外设电路**：
  - Type-C 接口 + USBLC6-2SC6 ESD 保护 + 5.1k CC 下拉
  - SWD 4-Pin 调试孔 (3V3, GND, SWDIO, SWCLK)
  - 接收机串口焊盘 (5V, GND, TX1, RX1)
  - 4 路电机信号焊盘 (M1: PB0, M2: PB1, M3: PB4, M4: PB5)
  - 电池电压检测分压电路 (10k + 1k 到 PC1/ADC1_IN11)
  - S8050 三极管蜂鸣器驱动电路 (PC15)
  - 状态 LED (PC13, PC14)

---

## 三、 PCB 投板前 DRC 检查清单

在导出 Gerber 投板前，务必逐项检查：
1. **IMU 物理位置**：是否位于板卡几何中心？下方底层是否有切断地平面或高频干扰线穿过？
2. **电源线宽**：5V 和 3.3V 主电源走线宽度是否 $\ge 15\text{mil} (0.38\text{mm})$？
3. **过孔尺寸**：嘉立创基础工艺标准（外径 0.5mm，孔径 0.3mm 或 0.45/0.2mm）。
4. **泪滴添加**：所有焊盘和过孔连接处是否已添加泪滴（Teardrops），增强机械抗撕裂能力。
