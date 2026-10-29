#ifndef __PERIPHERAL_INCLUDE_ST1VAFE3BX_H__
#define __PERIPHERAL_INCLUDE_ST1VAFE3BX_H__

#include <stdint.h>
#include <stdbool.h>
#include "ch58x_gpio.h"
#include "ch58x_spi.h"
#include "st1vafe3bx_reg.h"

#ifndef ST1VAFE3BX_SPI0_PORT
#define ST1VAFE3BX_SPI0_PORT SPI_PORT_0_0
#endif

#ifndef ST1VAFE3BX_SPI0_CLOCK_DIV
// CH585主频78MHz，8分频得到9.75MHz，低于ST1VAFE3BX手册SPC最大10MHz。
#define ST1VAFE3BX_SPI0_CLOCK_DIV 8
#endif

#ifndef ST1VAFE3BX_SPI0_MODE
#define ST1VAFE3BX_SPI0_MODE SPI_MODE_CPOL_0_CPHA_0
#endif

#ifndef ST1VAFE3BX_SPI0_ORDER
#define ST1VAFE3BX_SPI0_ORDER SPI_ORDER_MSB
#endif

#ifndef ST1VAFE3BX_SPI0_CS_CHANNEL
#define ST1VAFE3BX_SPI0_CS_CHANNEL SPI_0_CHANNEL_0
#endif

#ifndef ST1VAFE3BX_CH585_INT_GROUP
#define ST1VAFE3BX_CH585_INT_GROUP GPIO_GROUP_A
#endif

#ifndef ST1VAFE3BX_CH585_INT_PIN
#define ST1VAFE3BX_CH585_INT_PIN GPIO_PIN_1
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 输入输出参数标记宏定义 */
#define _IN_        const
#define _OUT_       
#define _INOUT_     volatile

/* 芯片核心固定参数定义 参考手册2.8/8.4节 */
#define ST1VAFE3BX_POWER_UP_MAX_DELAY   25000   // 上电命令最大等待时间25ms(单位us)
#define ST1VAFE3BX_VAFE_ACTIVE_DELAY    10000   // vAFE激活等待时间10ms(单位us)

/* 以下配置类型的取值宏统一来自 st1vafe3bx_reg.h */
typedef ST1VAFE3BX_FS ST1VAFE3BX_ACCEL_FS;
typedef ST1VAFE3BX_BW ST1VAFE3BX_ACCEL_BW;
typedef uint8_t ST1VAFE3BX_INTERRUPT_TRIGGER;

/**
 * @brief MLC动作配置方案编号
 * @note 上位机下发同一编号后，固件加载对应动作的MEMS Studio寄存器配置。
 */
typedef enum _ST1VAFE3BX_MLC_PROFILE_ID {
    ST1VAFE3BX_MLC_PROFILE_DF = 0U,
    ST1VAFE3BX_MLC_PROFILE_PF = 1U,
    ST1VAFE3BX_MLC_PROFILE_INV = 2U,
    ST1VAFE3BX_MLC_PROFILE_EV = 3U
} ST1VAFE3BX_MLC_PROFILE_ID;

/**
 * @brief 中断配置参数结构体
 */
typedef struct _ST1VAFE3BX_INTERRUPT_CONFIG {
    bool int_pin_enable;              // true=使能INT引脚输出，false=禁用
    bool timestamp_enable;     // true=使能时间戳，false=禁用时间戳
    bool basic_int_pin_enable;        // true=使能基本INT引脚输出，false=禁用，即各种检测中断
    bool level_interrupt_lir;  // true=锁存中断(LIR=1)，false=电平中断自动清除(LIR=0)
    bool int_fifo_threshold_enable;   // true=FIFO阈值中断源使能，false=禁用
    bool interrupt_polarity_active_low; // true=低电平有效，false=高电平有效
    bool od_output;            // true=开漏输出，false=推挽输出
} ST1VAFE3BX_INTERRUPT_CONFIG;

/**
 * @brief FIFO原始样本（7字节）：1字节TAG + 6字节数据
 * @note packed保持与FIFO字节流一致；数组首地址需4字节对齐以兼容DMA
 */
typedef struct __attribute__((packed)) {
    uint8_t tag;
    uint8_t data[6];
} ST1VAFE3BX_RAW_SAMPLE;

/**
 * @brief FIFO状态快照。
 * @note unread_words来自FIFO_STATUS2；watermark_asserted/overrun来自FIFO_STATUS1。
 */
typedef struct _ST1VAFE3BX_FIFO_STATUS {
    uint8_t unread_words;
    bool watermark_asserted;
    bool overrun;
} ST1VAFE3BX_FIFO_STATUS;

/**
 * @brief 一次FIFO DMA批量读取期间的诊断摘要。
 * @note 仅旁路记录状态，不改变FIFO数据内容和读取上限。
 */
typedef struct _ST1VAFE3BX_FIFO_READ_REPORT {
    bool status_observed;
    uint8_t max_unread_words;
    bool overrun_observed;
} ST1VAFE3BX_FIFO_READ_REPORT;

/**
 * @brief MLC feature FIFO原始字段拼接结果
 * @note TAG=1Ch时：X_L/X_H拼接为半精度浮点原始位，Y_L/Y_H拼接为feature identifier，
 *       Z_L/Z_H为保留字段；驱动层不执行half-float数值解算。
 */
typedef struct _ST1VAFE3BX_MLC_FEATURE_SAMPLE {
    uint8_t raw_tag;
    uint16_t value_bits;
    uint16_t feature_id;
    uint16_t reserved;
} ST1VAFE3BX_MLC_FEATURE_SAMPLE;

/**
 * @brief FIFO解码样本（加速度三轴 + vAFE）
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t vafe_data;
    uint32_t timestamp;
} ST1VAFE3BX_DECODED_SAMPLE;

/**
 * @brief vAFE配置参数结构体
 */
typedef struct _ST1VAFE3BX_VAFE_CONFIG {
    ST1VAFE3BX_VAFE_MODE mode;
    ST1VAFE3BX_VAFE_GAIN gain;
    ST1VAFE3BX_VAFE_ZIN input_impedance;
    bool vafe_only_mode; // true=仅vAFE工作模式，false=vAFE+加速度计同步模式
} ST1VAFE3BX_VAFE_CONFIG;

/**
 * @brief 加速度计配置参数结构体
 */
typedef struct _ST1VAFE3BX_ACCEL_CONFIG {
    ST1VAFE3BX_ACCEL_FS full_scale;      // 量程配置：2g/4g/8g/16g
    ST1VAFE3BX_ACCEL_BW bandwidth;       // 带宽配置：ODR/2, ODR/4, ODR/8, ODR/16
} ST1VAFE3BX_ACCEL_CONFIG;

/**
 * @brief FIFO配置参数结构体
 */
typedef struct _ST1VAFE3BX_FIFO_CONFIG {
    ST1VAFE3BX_FIFO_MODE mode;
    uint8_t watermark_threshold; // FIFO水印阈值，最大127
    bool accel_only_fifo; // 映射FIFO_WTM.XL_ONLY_FIFO位，true=仅XL(加速度)数据进入FIFO
    bool double_depth_mode; // 映射FIFO_CTRL.FIFO_DEPTH位，true=2倍深度模式
    ST1VAFE3BX_FIFO_BATCH_DEC_TS_BATCH timestamp_bdr;
    ST1VAFE3BX_FIFO_BATCH_DEC_BDR_XL accel_bdr;
} ST1VAFE3BX_FIFO_CONFIG;

/* ==================== 核心初始化与控制接口 ==================== */
#define ST1VAFE3BX_MLC_TREE_MAX                 (4U)

/**
 * @brief 执行 ST1VAFE3BX FIFO训练数据采集模式启动流程
 *        （SPI 初始化、芯片初始化、默认FIFO采集配置、开始采样）
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_startup_fifo_mode(void);

/**
 * @brief 执行 ST1VAFE3BX MLC分类模式启动流程
 *        （SPI 初始化、芯片初始化、vAFE/XL基础配置、加载MLC配置、中断路由、开始采样）
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_startup_mlc_mode(void);

/**
 * @brief 初始化ST1VAFE3BX驱动（执行上电与软复位流程）
 * @return 执行成功返回true，失败返回false
 * @note 该函数采用驱动内置底层SPI访问路径
 */
extern bool st1vafe3bx_init(void);

/**
 * @brief 仅初始化SPI0外设参数（不访问芯片寄存器）。
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_spi0_init(void);

/**
 * @brief 查询驱动是否已完成初始化
 * @return 已初始化返回true，未初始化返回false
 */
extern bool st1vafe3bx_is_initialized(void);

/**
 * @brief 执行芯片强制上电命令(初始化必选步骤)
 * @return 执行成功返回true，失败返回false
 * @note 芯片上电后必须执行该命令，才能访问寄存器 参考手册2.5.1节
 */
extern bool st1vafe3bx_power_up_cmd(void);

/**
 * @brief 发送软件复位命令，复位所有控制寄存器为默认值
 * @return 执行成功返回true，失败返回false
 * @note 通过CTRL1寄存器的SW_RESET位实现，复位后需重新执行上电命令 参考手册8.5节
 */
extern bool st1vafe3bx_soft_reset(void);

/**
 * @brief 应用芯片默认配置(加速度计2g+ODR/2带宽，vAFE差分x16增益+1GΩ，FIFO+中断使能)
 * @return 执行成功返回true，失败返回false
 * @note 此函数执行完整的硬件初始化序列，包括SPI访问使能、vAFE配置、FIFO配置与中断配置
 */
extern bool st1vafe3bx_apply_default_config(void);

/**
 * @brief 启动采样(使能加速度计/vAFE数据输出)
 * @return 执行成功返回true，失败返回false
 * @note 需先完成配置，通过CTRL5寄存器ODR位启动输出 参考手册8.9节
 */
extern bool st1vafe3bx_start(void);

/**
 * @brief 停止采样(进入掉电模式)
 * @return 执行成功返回true，失败返回false
 * @note 通过CTRL5寄存器ODR位写0000进入掉电模式 参考手册8.9节
 */
extern bool st1vafe3bx_stop(void);

/* ==================== 寄存器读写接口 ==================== */
/**
 * @brief 读取单个寄存器
 * @param reg 寄存器地址
 * @param value 输出寄存器读取值
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_register(
    _IN_ ST1VAFE3BX_REG reg,
    _OUT_ uint8_t *value);

/**
 * @brief DMA方式读取单个寄存器
 * @param reg 寄存器地址
 * @param value 输出寄存器读取值
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_register_dma(
    _IN_ ST1VAFE3BX_REG reg,
    _OUT_ uint8_t *value);

/**
 * @brief 写入单个寄存器
 * @param reg 寄存器地址
 * @param value 待写入寄存器的值
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_write_register(
    _IN_ ST1VAFE3BX_REG reg,
    uint8_t value);

/**
 * @brief 连续读取多个寄存器
 * @param start_reg 起始寄存器地址
 * @param data 输出数据缓冲区
 * @param length 读取数据长度
 * @return 执行成功返回true，失败返回false
 * @note 需使能CTRL1寄存器的IF_ADD_INC位(默认使能)，地址自动递增 参考手册8.5节
 */
extern bool st1vafe3bx_read_registers(
    _IN_ ST1VAFE3BX_REG start_reg,
    _OUT_ uint8_t *data,
    uint8_t length);

/**
 * @brief DMA方式连续读取多个寄存器
 * @param start_reg 起始寄存器地址
 * @param data 输出数据缓冲区
 * @param length 读取数据长度
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_registers_dma(
    _IN_ ST1VAFE3BX_REG start_reg,
    _OUT_ uint8_t *data,
    uint16_t length);

/**
 * @brief 连续写入多个寄存器
 * @param start_reg 起始寄存器地址
 * @param data 待写入数据缓冲区
 * @param length 写入数据长度
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_write_registers(
    _IN_ ST1VAFE3BX_REG start_reg,
    _IN_ uint8_t *data,
    uint8_t length);

/* ==================== 芯片状态与ID接口 ==================== */
/**
 * @brief 读取芯片ID(WHO_AM_I寄存器)
 * @param who_am_i 输出芯片ID值
 * @return 执行成功返回true，失败返回false
 * @note 芯片固定ID为0x48，可用于校验通信与芯片身份 参考手册8.4节
 */
extern bool st1vafe3bx_read_who_am_i(_OUT_ uint8_t *who_am_i);

/**
 * @brief 初始化CH585 PA1为ST1VAFE3BX中断输入并使能GPIOA中断
 * @return 初始化成功返回true，失败返回false
 */
extern bool st1vafe3bx_init_interrupt_input(void);

/**
 * @brief 查询是否收到ST1VAFE3BX中断事件
 * @return 有中断事件返回true，无事件返回false
 */
extern bool st1vafe3bx_is_interrupt_pending(void);

/**
 * @brief 查询ST1VAFE3BX中断脚当前电平（FIFO水位条件是否成立）
 * @return 引脚为高（未读样本数已达水位FTH）返回true，否则返回false
 */
extern bool st1vafe3bx_is_fifo_watermark_asserted(void);

/**
 * @brief 清除ST1VAFE3BX中断事件标志
 */
extern void st1vafe3bx_clear_interrupt_pending(void);

/**
 * @brief 按FIFO推荐流程处理中断并读取FIFO数据直到为空
 * @param samples 输出FIFO样本缓冲区
 * @return 处理成功返回true，失败返回false
 */
extern bool st1vafe3bx_handle_interrupt_fifo(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *raw_samples,
    _OUT_ ST1VAFE3BX_DECODED_SAMPLE *decoded_samples);

/**
 * @brief 处理中断并按当前FIFO计数执行一次DMA批量读取+解码
 * @param samples 输出FIFO样本缓冲区
 * @return 处理成功返回true，失败返回false
 */
extern bool st1vafe3bx_handle_interrupt_fifo_dma(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *raw_samples,
    _OUT_ ST1VAFE3BX_DECODED_SAMPLE *decoded_samples);

/* ==================== 核心配置接口 ==================== */
/**
 * @brief 配置加速度计参数（支持FS、带宽、ODR、高性能模式、BDU）
 * @param config 加速度计配置参数结构体指针
 * @return 执行成功返回true，失败返回false
 * @note 此函数仅配置FS和BDU；ODR由st1vafe3bx_start/stop统一控制
 */
extern bool st1vafe3bx_config_accel(_IN_ ST1VAFE3BX_ACCEL_CONFIG *config);

/**
 * @brief 配置中断输出特性（INT引脚、中断源、中断模式等）
 * @param config 中断配置参数结构体指针
 * @return 执行成功返回true，失败返回false
 * @note 此函数独立于应用默认配置，提供灵活的中断参数设置
 */
extern bool st1vafe3bx_config_interrupt(_IN_ ST1VAFE3BX_INTERRUPT_CONFIG *config);

/**
 * @brief 配置vAFE生物电势通道参数（模式、增益、阻抗等）
 * @param config vAFE配置参数结构体指针
 * @return 执行成功返回true，失败返回false
 * @note 此函数独立于应用默认配置，提供灵活的参数设置
 */
extern bool st1vafe3bx_config_vafe(_IN_ ST1VAFE3BX_VAFE_CONFIG *config);

/**
 * @brief 配置FIFO参数（模式、水位、深度模式等）
 * @param config FIFO配置参数结构体指针
 * @return 执行成功返回true，失败返回false
 * @note 此函数独立于应用默认配置，提供灵活的参数设置
 */
extern bool st1vafe3bx_config_fifo(_IN_ ST1VAFE3BX_FIFO_CONFIG *config);

/**
 * @brief 加载由 MEMS Studio 导出的 MLC 配置序列
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_apply_mlc_config(void);

/**
 * @brief 选择当前MLC动作配置方案
 * @param profile_id 动作配置编号
 * @return 找到对应配置返回true，否则返回false
 */
extern bool st1vafe3bx_set_mlc_profile(ST1VAFE3BX_MLC_PROFILE_ID profile_id);

/**
 * @brief 获取当前MLC动作配置方案编号
 * @return 当前动作配置编号
 */
extern ST1VAFE3BX_MLC_PROFILE_ID st1vafe3bx_get_mlc_profile(void);

/**
 * @brief 停止采样并重新加载指定MLC动作配置
 * @param profile_id 动作配置编号
 * @return 重新加载并启动成功返回true，否则返回false
 */
extern bool st1vafe3bx_reload_mlc_profile(ST1VAFE3BX_MLC_PROFILE_ID profile_id);

/**
 * @brief 配置 MLC1 输出状态改变事件到 INT 引脚
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_config_mlc_interrupt(void);

/**
 * @brief 根据MEMS Studio导出的MLC元数据获取当前配置包含的决策树掩码
 * @return bit0~bit3分别表示MLC1~MLC4；返回0表示当前配置未导出MLC输出
 */
extern uint8_t st1vafe3bx_get_mlc_tree_mask(void);

/**
 * @brief 读取MLC输出变化事件状态掩码
 * @param status_mask 输出MLC_STATUS_MAINPAGE低4位，bit0~bit3对应MLC1~MLC4
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_mlc_status_mask(_OUT_ uint8_t *status_mask);

/**
 * @brief 读取指定MLC决策树分类结果
 * @param tree_id 决策树编号，范围1~4，对应MLC1~MLC4
 * @param result 输出MLCx_SRC原始结果
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_mlc_result(uint8_t tree_id, _OUT_ uint8_t *result);

/**
 * @brief 一次embedded page会话读取多个MLC决策树分类结果
 * @param tree_mask bit0~bit3分别表示需要读取的MLC1~MLC4
 * @param results 输出数组，长度至少为ST1VAFE3BX_MLC_TREE_MAX；索引0~3对应MLC1~MLC4
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_mlc_results(
    uint8_t tree_mask,
    _OUT_ uint8_t *results);

/**
 * @brief 读取并校验MLC结果，失败时自动重试一次
 * @param tree_mask bit0~bit3分别表示需要读取的MLC1~MLC4
 * @param results 输出数组，长度至少为ST1VAFE3BX_MLC_TREE_MAX
 * @return 结果全部有效返回true，否则返回false
 */
extern bool st1vafe3bx_read_valid_mlc_results(
    uint8_t tree_mask,
    _OUT_ uint8_t *results);

/**
 * @brief 判断MLC分类结果是否存在于当前动作配置导出的结果表中
 * @param tree_id 决策树编号，范围1~4，对应MLC1~MLC4
 * @param result MLCx_SRC原始结果值
 * @return 匹配到有效结果返回true，否则返回false
 */
extern bool st1vafe3bx_is_mlc_result_valid(uint8_t tree_id, uint8_t result);

/**
 * @brief 启动MLC feature写入FIFO
 * @param fifo_config 普通FIFO配置参数
 * @return 执行成功返回true，失败返回false
 * @note 该函数会配置普通FIFO路径、FIFO advanced路径，并在embedded page打开
 *       MLC_FILTER_FEATURE_FIFO_EN，使TAG=1Ch feature包进入FIFO。
 */
extern bool st1vafe3bx_start_mlc_feature_fifo(
    _IN_ const ST1VAFE3BX_FIFO_CONFIG *fifo_config);

/**
 * @brief 主动读取FIFO中的MLC feature包并完成字段拼接
 * @param features 输出feature样本缓冲区
 * @param feature_capacity 输出缓冲区可容纳的feature数量
 * @param feature_count 输出实际解析到的feature数量
 * @param raw_sample_count 输出本次读取的FIFO原始样本数量，可为NULL
 * @return 执行成功返回true；若输出缓冲区不足会继续清空FIFO并返回false
 * @note 仅接收TAG=1Ch的MLC feature包，驱动层只拼接字段，不执行half-float解算。
 */
extern bool st1vafe3bx_read_mlc_feature_fifo(
    _OUT_ ST1VAFE3BX_MLC_FEATURE_SAMPLE *features,
    uint16_t feature_capacity,
    _OUT_ uint16_t *feature_count,
    _OUT_ uint16_t *raw_sample_count);

/* ==================== 数据读取接口 ==================== */
/**
 * @brief 读取加速度计原始数据(XYZ三轴)
 * @param x 输出X轴加速度数据
 * @param y 输出Y轴加速度数据
 * @param z 输出Z轴加速度数据
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_accel_data(_OUT_ int16_t *x, _OUT_ int16_t *y, _OUT_ int16_t *z);

/**
 * @brief 读取vAFE生物电势原始数据
 * @param vafe_data 输出vAFE 12位原始数据
 * @return 执行成功返回true，失败返回false
 * @note 数据为12位二进制补码，左对齐格式 参考手册2.2节
 */
extern bool st1vafe3bx_read_vafe_data(_OUT_ int16_t *vafe_data);

/**
 * @brief 从FIFO读取单个原始样本（仅读取，不解码）
 * @param sample 输出FIFO原始样本
 * @return 读取成功返回true，FIFO空/读取失败返回false
 */
extern bool st1vafe3bx_read_fifo_sample(_OUT_ ST1VAFE3BX_RAW_SAMPLE *sample);

/**
 * @brief 从FIFO读取单个原始样本（DMA模式，仅读取，不解码）
 * @param sample 输出FIFO原始样本
 * @return 读取成功返回true，FIFO空/读取失败返回false
 */
extern bool st1vafe3bx_read_fifo_sample_dma(_OUT_ ST1VAFE3BX_RAW_SAMPLE *sample);

/**
 * @brief 从FIFO批量读取多个原始样本（仅读取，不解码）
 * @param samples 输出FIFO原始样本缓冲区
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_fifo_burst(
     _OUT_ ST1VAFE3BX_RAW_SAMPLE *samples);

/**
 * @brief 从FIFO批量读取多个原始样本（DMA模式，仅读取，不解码）
 * @param samples 输出FIFO原始样本缓冲区
 * @param sample_count 输出实际读取的样本数量
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_fifo_burst_dma(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *samples,
    _IN_ uint16_t max_samples,
    _OUT_ uint16_t *sample_count);

/**
 * @brief 从FIFO批量读取多个原始样本，同时返回读取期间的FIFO状态摘要。
 * @param samples 输出FIFO原始样本缓冲区
 * @param max_samples 调用方缓冲区最多可容纳的样本数
 * @param sample_count 输出实际读取的样本数量
 * @param report 输出本次读取期间的FIFO诊断摘要
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_read_fifo_burst_dma_with_report(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *samples,
    _IN_ uint16_t max_samples,
    _OUT_ uint16_t *sample_count,
    _OUT_ ST1VAFE3BX_FIFO_READ_REPORT *report);

/**
 * @brief 对单个原始FIFO样本进行解码
 * @param raw_sample 输入原始样本
 * @param decoded_sample 输出解码样本
 * @return 解码成功返回true，失败返回false
 */
extern bool st1vafe3bx_decode_fifo_sample(
    _IN_ const ST1VAFE3BX_RAW_SAMPLE *raw_sample,
    _OUT_ ST1VAFE3BX_DECODED_SAMPLE *decoded_sample);

/**
 * @brief 对批量原始FIFO样本进行解码
 * @param raw_samples 输入原始样本缓冲区
 * @param decoded_samples 输出解码样本缓冲区
 * @param sample_count 样本数量
 * @return 解码成功返回true，失败返回false
 */
extern bool st1vafe3bx_decode_fifo_burst(
     _IN_ const ST1VAFE3BX_RAW_SAMPLE *raw_samples,
     _OUT_ ST1VAFE3BX_DECODED_SAMPLE *decoded_samples,
    uint16_t sample_count);

/**
 * @brief 读取FIFO当前存储的样本数量
 * @param sample_count 输出FIFO内未读样本数
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_get_fifo_count(_OUT_ uint8_t *sample_count);

/**
 * @brief 一次连续读取FIFO_STATUS1/2，获得水位、WTM和OVR状态
 * @param status 输出FIFO状态快照
 * @return 执行成功返回true，失败返回false
 */
extern bool st1vafe3bx_get_fifo_status(_OUT_ ST1VAFE3BX_FIFO_STATUS *status);

#ifdef __cplusplus
}
#endif

#endif // __PERIPHERAL_INCLUDE_ST1VAFE3BX_H__
