# 自研飞控硬件原理图与 PCB 设计规范 (STM32F405)

本文档定义自研飞控（FC）的引脚功能分配表（Pinout Mapping）、电路子系统原理图要点与 4 层板 PCB 布局布线避坑准则。

---

## 一、 STM32F405RGT6 引脚资源分配（Pinout Table）

为了保证 DMA 传输与定时器资源的最高效率，引脚经过了精密规划（特别是 4 路电机共用同一个硬件定时器 TIM3，以完美支持 DShot DMA 突发传输）：

| 引脚编号 (LQFP64) | 网络标号 | 功能复用 | 连接目标器件 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| **Pin 21, 22, 23** | SPI1_SCK / MISO / MOSI | PA5, PA6, PA7 | **ICM-42688-P** (SPI) | 飞控最核心高速数据链路，配置为 10MHz~20MHz |
| **Pin 24** | IMU_CS | PC4 (GPIO Output) | **ICM-42688-P** 片选 CS | 软件片选控制 |
| **Pin 25** | IMU_INT | PC5 (EXTI5 中断) | **ICM-42688-P** 数据就绪 INT | 硬件中断触发 1kHz 姿态解算，杜绝轮询延迟 |
| **Pin 56, 57** | TIM3_CH1, TIM3_CH2 | PB4, PB5 | **电机 M3, M4 信号焊盘** | 电机 3、电机 4 输出 (支持 DShot300/600 & PWM) |
| **Pin 26, 27** | TIM3_CH3, TIM3_CH4 | PB0, PB1 | **电机 M1, M2 信号焊盘** | 电机 1、电机 2 输出 (与 M3/M4 同属 TIM3) |
| **Pin 42, 43** | USART1_TX, USART1_RX | PA9, PA10 | **ELRS / CRSF 接收机** | 遥控器接收机通信串口 (420000 波特率) |
| **Pin 58, 59** | I2C1_SCL, I2C1_SDA | PB6, PB7 | **SPL06-001 气压计** | 定高传感器总线 (标准 400kHz Fast Mode) |
| **Pin 16** | VBAT_ADC | PC1 (ADC1_IN11) | **动力电池分压网络** | 10k:1k 分压检测电池实时电压 |
| **Pin 17** | CURR_ADC | PC2 (ADC1_IN12) | **4合1电调电流计** | 采样电调模拟电流输出，计算瞬时功耗与已消耗电量 |
| **Pin 7** | USB_DM | PA11 (USB_FS_DM) | **Type-C 接口 D-** | USB 虚拟串口 / DFU 下载 |
| **Pin 44** | USB_DP | PA12 (USB_FS_DP) | **Type-C 接口 D+** | USB 虚拟串口 / DFU 下载 |
| **Pin 46, 49** | SWDIO, SWCLK | PA13, PA14 | **4-Pin SWD 调试孔** | 连接 DAP-Link / ST-Link 烧录与在线 Debug |
| **Pin 2** | LED_BLUE | PC13 (GPIO Output) | **状态指示灯 (蓝)** | 系统心跳、ARM 解锁指示 |
| **Pin 3** | LED_GREEN | PC14 (GPIO Output) | **状态指示灯 (绿)** | 传感器自检通过与对频状态 |
| **Pin 4** | BUZZER_PIN | PC15 (GPIO Output) | **S8050 三极管基极** | 控制有源蜂鸣器发声 |
| **Pin 12, 13** | OSC_IN, OSC_OUT | PH0, PH1 | **8MHz 无源晶振** | 外接 8MHz 晶振输入 |
| **Pin 60** | BOOT0 | BOOT0 | **轻触按键 / 下拉电阻** | 按住上电拉高至 3.3V 进入 Bootloader 模式 |
| **Pin 7** | NRST | NRST | **100nF 电容到地 + 按键**| 芯片硬件复位引脚 |

---

## 二、 关键电路原理图设计要点

### 1. IMU 高性能抗干扰电路
* **电源滤波**：在 ICM-42688 的 VDD 与 VDDIO 前，加入一颗磁珠（600Ω@100MHz）以及 `0.1μF + 2.2μF` 低 ESR 贴片陶瓷电容。
* **去耦距离**：滤波电容必须**紧靠传感器芯片引脚**，引脚到电容焊盘的距离控制在 1.5mm 以内。
* **SPI 总线串联阻抗匹配**：在 SCK、MOSI 线上可预留 22Ω~33Ω 串联小电阻，用于消除高速时钟边沿振铃。

### 2. 动力电池精准分压采样电路
* 采用 `10kΩ 1% (R1)` 与 `1.0kΩ 1% (R2)` 组成分压网络：
  $$V_{ADC} = V_{BAT} \times \frac{R_2}{R_1 + R_2} = V_{BAT} \times \frac{1}{11}$$
  当接入 4S 满电电池（16.8V）时，$V_{ADC} = 16.8\text{V} / 11 \approx 1.527\text{V}$，完全落在 STM32 ADC 的 0~3.3V 安全量程内。
* **滤波**：分压输出端并联一颗 100nF 贴片电容到模拟地，滤除高频电机换向噪声。

### 3. Type-C 保护与 DFU 电路
* CC1、CC2 各接一颗 5.1kΩ 下拉电阻到地，确保标准 PD 充电头或电脑 Type-C 接口能够正确识别供电。
* D+、D- 串入 USBLC6-2SC6 静电保护阵列，严防手触接口造成 MCU 击穿。

---

## 三、 四层板 PCB 叠层与布线避坑准则

```
Layer 1 (Top Signal)   : 核心元器件、MCU、IMU、高速信号走线 (SPI / USB)
Layer 2 (Inner GND)    : 完整未被切割的连续地平面 (Solid Ground Plane)
Layer 3 (Inner Power)  : 3.3V 模拟/数字电源走线与局部电源铺铜
Layer 4 (Bottom Signal): 焊接焊盘、接口走线、次要低速控制线
```

### 1. IMU 物理布局黄金准则
1. **几何中心放置**：IMU 必须尽可能放置在飞控板的中心交叉线上，旋转轴心偏移越小，姿态解算误差越小。
2. **底层开窗与静音区（Keep-out Zone）**：
   * 在 IMU 正下方的 Top 层，铺铜需打孔至 Layer 2 地平面；
   * **严禁**在 IMU 正下方走任何电机信号线、晶振时钟线和大电流电源线！
3. **结构减震设计**：安装孔必须设计为 M2 通孔，安装在机架上时**必须加装硅胶减震球（Grommet）**，避免机臂的高频震动直接硬传导给 PCB。

### 2. 晶振布局布线规范
* 8MHz 晶振与其两颗 18pF 负载电容需形成紧凑回路，走线尽量短且对称。
* 晶振区域在 Layer 1 用地线包围（Guard Ring），禁止其他走线从晶振下方穿过。

### 3. 地平面完整性（GND Integrity）
* Layer 2 必须作为**绝对完整的参考地平面**，尽量不在 Layer 2 走任何信号走线。所有的元器件就近打过孔连接到 Layer 2。
* 任何高频信号线（SPI、USB、DShot）的回流路径都在其紧邻的接地平面上，地平面完整能最大幅度降低 EMI 辐射与串扰。
