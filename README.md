# CH585 与 ST1VAFE3BX 穿戴传感器开发

本仓库整理了基于 CH585 与 ST1VAFE3BX 的可穿戴 sEMG-IMU 节点代码：CH585 通过 SPI 读取传感器的加速度计与 vAFE 数据，并通过 BLE 向上位机发送样本或 MLC 分类结果。代码从已有的 CH585 工程中提取，保留原有目录结构，方便对照和集成。

## 工作流程

```text
ST1VAFE3BX 加速度计 / vAFE
        │ FIFO、时间戳与中断
        ▼
CH585 SPI / DMA → FIFO 解码 → 帧打包与批量发送 → BLE Data Pipe → 上位机
        └──────── MLC 决策树结果 ────────────────┘
```

- **FIFO 训练模式**：采集加速度与 vAFE 样本，按 FIFO 水位读取并通过 BLE Notification 上行。
- **MLC 分类模式**：加载对应配置，在结果变化时读取并上报 MLC 决策树输出。
- 上位机可发送控制帧切换工作模式或 MLC 配置；具体逻辑见 `main.c`。

原工程还包含 BLE 栈、板级支持、上位机与 IAP 工程。本仓库仅提取下列四个源码文件，并不包含这些完整组件。

## 代码内容

```text
CH585/
├── user/main.c                         # 应用入口、采集模式和 BLE 数据发送
└── peripheral/
    ├── st1vafe3bx.c                    # 芯片配置、SPI、FIFO 和 MLC 驱动
    └── include/
        ├── st1vafe3bx.h                # 驱动接口和平台配置
        └── st1vafe3bx_reg.h            # 寄存器与字段定义
```

驱动包含 SPI 访问、FIFO 批量读取和解码、传感器中断及 MLC 配置与结果读取。应用示例包含 FIFO 采集模式、MLC 分类模式和通过 BLE Data Pipe 发送数据。具体支持范围以源码为准。

## 集成和构建

这是一份**源码提取项目，不是独立可编译的完整固件工程**。需要另行提供原 CH585 工程或兼容的 WCH SDK、板级支持、BLE 协议栈与 Data Pipe 服务、启动文件、链接脚本和 RISC-V 工具链。`main.c` 还依赖 `board.h`、`hal.h`、`ble_peripheral.h`、`data_pipe_service.h`、`state_led.h` 等工程组件；驱动依赖 CH58x GPIO/SPI/时钟接口。

`st1vafe3bx.c` 还引用 `mlc_DF.h`、`mlc_PF.h`、`mlc_INV.h`、`mlc_EV.h`。这些由 MEMS Studio 生成的配置文件带有 STMicroelectronics 版权声明，其原始许可文件未包含在本源码提取中；请从有权使用的原工程或工具输出中提供，并确认适用许可。不要用空文件替代，否则 MLC 模式不可用。

可在已有的 CH585 MounRiver Studio 工程中，备份同名文件后按以上目录放置四个文件，并确保上述依赖和模型头文件在编译器包含路径中。原工程使用 CH585M 和 WCH RISC-V GCC 12；工程的具体版本、硬件接线及配置应与目标板核对。

## 硬件和数据接口

- 驱动默认使用 SPI0、模式 0、MSB 先行、时钟分频 8；默认传感器中断输入为 PA1。这些宏定义在 `st1vafe3bx.h` 中，可按实际硬件配置。
- FIFO 默认配置使用 800 Hz ODR、连续模式和 64 个 FIFO 字的水位阈值，时间戳字与数据字均计入水位。FIFO 满时可能覆盖旧数据。
- 默认启动 FIFO 训练模式；可在构建时设置 `ST1_DEFAULT_MODE` 选择 MLC 分类模式。模式与配置切换逻辑见 `main.c`。
- 默认 vAFE 增益为 16×。采用项目架构说明中的典型换算时，左对齐的有符号 16 位 vAFE 值可除以 1311 得到近似 mV；实际使用应依据器件手册和硬件校准。

## BLE 数据格式

原工程的 Data Pipe 服务使用 `0xFFC0`，命令写入特征为 `0xFFC1`，数据通知特征为 `0xFFC2`。服务实现不包含在本仓库中。

```text
采样： A5 5B | TS3 TS2 TS1 TS0 | P0 P1 P2 P3 P4 P5
分类： C1 | tree_id | raw_result
控制： D0 | cmd | param
应答： E0 | cmd | status | mode | detail
```

采样帧为 12 字节。时间戳为 4 字节大端原始值，单位为 10 µs；6 字节数据区依次打包 X、Y、Z、vAFE 四个 12 位补码。帧打包逻辑见 `stream_pack_sample_to_frame()`；与旧版 `A5 5A` 14 字节协议不能直接混用。控制命令 `0x01` 切换 DF/PF/INV/EV MLC 配置，`0x02` 切换 FIFO/MLC 模式。

通知批次根据协商 MTU 计算，最多 40 帧；发送失败时保留批次并重试。链路拥塞时 FIFO 仍可能溢出，丢失时间戳的样本可能显示为零。原工程历史多节点实验的结果受固件版本、硬件和链路条件限制，不能作为本提取项目的性能保证。

## 验证状态

本地使用 WCH RISC-V GCC 12，以原工程的编译参数、头文件和 MLC 配置为依赖，对提取的 `main.c` 和 `st1vafe3bx.c` 分别执行 `-fsyntax-only`，均通过。此检查没有链接固件，也没有验证独立构建或实际硬件上的 BLE、时序、采样精度和 MLC 模型效果。

## 许可

本仓库的许可见根目录 `LICENSE`（MIT）。CH585 SDK、BLE 组件和 MEMS Studio 生成文件各自的版权与许可独立适用，集成或再分发前请核对其来源和许可。
