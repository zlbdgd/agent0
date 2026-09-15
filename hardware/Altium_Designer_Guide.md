# Altium Designer (AD) 飞控硬件制版实战指南

本文档为使用 **Altium Designer (AD)** 设计自研 STM32F405 四层飞控板提供全套工程规范、规则约束（Design Rules）、叠层管理与制造输出指导。

---

## 一、 AD 工程结构推荐

在 `hardware/` 目录下建立标准的 AD 硬件工程：

```
hardware/
├── FlightController.PrjPcb             # AD 核心工程文件
├── Schematics/                         # 原理图图纸 (.SchDoc)
│   ├── 01_Power_Supply.SchDoc          # 5V稳压、3.3V LDO与电源滤波
│   ├── 02_MCU_Minimum_System.SchDoc    # STM32F405 核心系统、晶振与复位
│   ├── 03_Sensors_IMU.SchDoc           # ICM-42688 高速 SPI 传感器电路
│   └── 04_Interfaces_IO.SchDoc         # Type-C、电调焊盘、接收机与蜂鸣器
├── PCB/
│   └── FlightController_20x20.PcbDoc   # 4层 PCB 布局布线文件
├── Libraries/                          # 本地集成库 (元器件库与封装库)
│   ├── FlightController.SchLib         # 原理图符号库
│   └── FlightController.PcbLib         # PCB 封装与 3D STEP 模型库
└── Outputs/                            # 投板生产制造文件 (Gerber/Drill/BOM)
```

---

## 二、 4 层板叠层管理器配置 (Layer Stack Manager)

按快捷键 `D + K` 打开 **Layer Stack Manager**，采用针对高频飞控抗震与低噪声最优的 4 层对称结构：

| 板层名称 | 类型 (Type) | 材料与厚度参考 | 铜箔厚度 | 铺铜与走线规划 |
| :--- | :--- | :--- | :--- | :--- |
| **Top Layer** | Signal | 铜箔 + 阻焊 (0.035mm) | 1 oz (35μm) | 放置 MCU、IMU、LDO，走高灵敏信号与 SPI 总线 |
| **Mid-Layer 1** | **Internal Plane / Signal** | 芯板/半固化片 Prepreg 7628 | 1 oz / 0.5 oz | **完整接地平面 (GND)**，作为顶层高频回流低阻抗参考平面 |
| **Mid-Layer 2** | **Internal Plane / Signal** | Core 0.8mm~1.0mm | 1 oz / 0.5 oz | **主电源层 (PWR)**：3.3V 洁净铺铜与 5V 主供电回路 |
| **Bottom Layer**| Signal | 铜箔 + 阻焊 (0.035mm) | 1 oz (35μm) | 连接外部电调的大焊盘、次要低速控制线与地铺铜 |

> **板厚设置**：整板成品厚度推荐选 **1.0mm** 或 **1.2mm**（比常规 1.6mm 更轻，20x20mm 微型穿越机对机载减重极敏感）。

---

## 三、 AD 关键设计规则配置 (PCB Rules & Constraints)

按快捷键 `D + R` 进入 **PCB Rules and Constraints Editor** 进行工厂级规则设置（适配嘉立创等高可靠工艺）：

### 1. 安全间距规则 (Electrical Clearance)
* **默认信号间距**：`Clearance_Default` $\ge 6.0\text{mil} (0.152\text{mm})$。
* **高灵敏/大电流网络**：新建规则针对 `InNetClass('PWR')` 或焊盘到铺铜间距设为 $\ge 8.0\text{mil}$。

### 2. 走线线宽规则 (Routing Width)
在原理图中建立 Net Class，并映射到 AD 走线规则：
* **普通信号线 (Default)**：最小 `6 mil`，推荐 `8 mil`，最大 `12 mil`。
* **主电源线 (InNetClass('PWR'))**：
  * 3.3V 主干电源：推荐 `15 mil ~ 20 mil`。
  * 5V 及电池电压输入：推荐 `20 mil ~ 30 mil`。
* **SPI 高速总线 (SCK/MOSI/MISO)**：固定推荐 `6 mil ~ 8 mil`，走线尽量等长且保持伴随地线。

### 3. 过孔规格规则 (Routing Via Style)
* **通孔 (Through-Hole Via)**：
  * 孔径 (Hole Size)：`12 mil (0.3mm)`。
  * 外径 (Diameter)：`20 mil (0.5mm)` 或 `24 mil (0.6mm)`。
  * *（注意：MCU 核心地打过孔时可选用 0.3mm/0.5mm，电源换层过孔建议双孔并联降低寄生电感）*。

### 4. 差分对规则 (Differential Pairs - 用于 Type-C USB)
在原理图中给 `USB_DP` 和 `USB_DM` 网络添加 **Differential Pair 指示标号**（命名为 `USB`）：
* 差分线宽：`6 mil`。
* 差分对间距：`6 mil`（在 4 层板叠层下大约对应 90Ω 差分阻抗）。

### 5. 铺铜连接规则 (Polygon Connect Style)
* **普通元器件接地引脚**：Relief Connect（十字花焊盘），导线宽度 `10 mil`，便于电烙铁焊接防止散热过快。
* **过孔 (Via)**：Direct Connect（全连接直通铺铜），降低接地阻抗。

---

## 四、 原理图与 PCB 库获取技巧（避免重复造轮子）

1. **立创元器件库直接导入 AD**：
   * 打开 [立创开源硬件平台 / 立创商城](https://www.szlcsc.com/)，搜索 STM32F405RGT6 (C15840) 或 ICM-42688-P (C2838382)。
   * 点击 **“导出到 Altium Designer”**，即可一键下载包含**原理图符号库 (.SchLib)**、**PCB封装库 (.PcbLib)** 和 **精准 3D STEP 模型** 的文件，直接拖入 AD 工程即可使用！
2. **AD 官方 Manufacturer Part Search**：
   * 在 AD 右侧面板打开 **Manufacturer Part Search**，直接搜索阻容感元器件型号，右键即可直接 Place 到原理图上。

---

## 五、 AD 布局布线核心操作规范

### 1. 物理板框绘制
1. 切换到 **Mechanical 1** 机械层。
2. 绘制 `20mm × 20mm` 正方形外框，倒圆角 `R = 1.2mm`。
3. 四角放置焊盘作为螺丝孔：外径 `3.5mm`，孔径 `2.1mm`（M2 标准通孔），四孔中心距严格为 `20.0mm × 20.0mm`。
4. 全选机械框线，按菜单栏：**Design -> Board Shape -> Define from Selected Objects**（快捷键 `D + S + D`）生成板卡轮廓。

### 2. IMU (ICM-42688) 专属布局守则
* **严格居中**：将 ICM-42688-P 封装对准板卡正中几何原点。
* **滤波电容紧贴**：VDD 与 VDDIO 的 0.1μF 去耦电容紧挨芯片管脚，地焊盘直接就近打 0.3mm 过孔直达 Mid-Layer 1 (GND)。
* **静音净空区**：在 IMU 正下方的 Top Layer，严禁走任何其他信号走线；Mid-Layer 1 保持完整无挖空接地。

---

## 六、 生产制造文件导出 (Gerber 投板流程)

画板完成并通过 DRC（快捷键 `T + D`）无报错后，按以下步骤导出工厂生产文件：

1. **导出 Gerber**：
   * 菜单栏 **File -> Fabrication Outputs -> Gerber Files**。
   * **General**：选择 Inches，格式 `2:5`。
   * **Layers**：勾选 `Top/Bottom Layer`、`Mid1/Mid2 Layer`、`Top/Bottom Solder`（阻焊层）、`Top/Bottom Overlay`（丝印层）。
2. **导出 NC Drill (数控钻孔)**：
   * 菜单栏 **File -> Fabrication Outputs -> NC Drill Files**。
   * 格式与 Gerber 保持一致（Inches，`2:5`）。
3. **坐标文件 (若选择 SMT 贴片)**：
   * **File -> Assembly Outputs -> Generates pick and place files**，导出 CSV 坐标文件。
4. 将生成的 Gerber 与 Drill 文件打包为 `.zip`，即可直接上传至嘉立创等制版厂进行高精度打样！
