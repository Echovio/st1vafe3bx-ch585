#ifndef __PERIPHERAL_INCLUDE_ST1VAFE3BX_REG_H__
#define __PERIPHERAL_INCLUDE_ST1VAFE3BX_REG_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ST1VAFE3BX_SPI_READ_FLAG        (0x01)  // 读操作：bit7置1
#define ST1VAFE3BX_SPI_WRITE_FLAG       (0x00)  // 写操作：bit7置0

/**
 * @brief SPI 接口固定参数
 * 命令字格式：bit7=RW(1读0写)，bit[6:0]=7位寄存器地址
 */
#define ST1VAFE3BX_SPI_ADDR_MASK        (0x7F)         // 7位寄存器地址掩码
#define ST1VAFE3BX_SPI_ADDR_POS         (0U)           // 地址放在bit[6:0]
#define ST1VAFE3BX_SPI_RW_BIT_POS       (7U)           // RW位放在bit7
#define ST1VAFE3BX_SPI_RW_BIT_MASK      (0x01U)        // RW值掩码
#define ST1VAFE3BX_REG_MAX_ADDR         (0xD2U)        //寄存器最大地址
#define ST1VAFE3BX_SPI_BUILD_CMD(addr, rw) \
    (uint8_t)((((uint8_t)(rw) & ST1VAFE3BX_SPI_RW_BIT_MASK) << ST1VAFE3BX_SPI_RW_BIT_POS) | \
              ((uint8_t)(addr) & ST1VAFE3BX_SPI_ADDR_MASK))

/******************************************************************************
 * 2. 芯片固定标识常量
 * ：8.4 WHO_AM_I (0Fh) (page37)
 *****************************************************************************/
#define ST1VAFE3BX_WHO_AM_I_FIXED_VALUE (0x48)  // WHO_AM_I寄存器固定返回值

/******************************************************************************
 * Register mapping
 *****************************************************************************/
typedef enum _ST1VAFE3BX_REG {
    ST1VAFE3BX_REG_EXT_CLK_CFG         = 0x08,  // 外部时钟配置寄存器
    ST1VAFE3BX_REG_PIN_CTRL            = 0x0C,  // 引脚控制寄存器
    ST1VAFE3BX_REG_WAKE_UP_DUR_EXT     = 0x0E,  // 唤醒时长扩展寄存器
    ST1VAFE3BX_REG_WHO_AM_I            = 0x0F,  // 设备ID寄存器
    ST1VAFE3BX_REG_CTRL1               = 0x10,  // 控制寄存器1
    ST1VAFE3BX_REG_CTRL2               = 0x11,  // 控制寄存器2
    ST1VAFE3BX_REG_CTRL3               = 0x12,  // 控制寄存器3
    ST1VAFE3BX_REG_CTRL4               = 0x13,  // 控制寄存器4
    ST1VAFE3BX_REG_CTRL5               = 0x14,  // 控制寄存器5（ODR/量程配置）
    ST1VAFE3BX_REG_INTERRUPT_CFG       = 0x17,  // 中断配置寄存器，含时间戳使能
    ST1VAFE3BX_REG_SIXD                = 0x18,  // 6D/4D方向检测配置寄存器

    // 唤醒/自由落体/轻击配置寄存器
    ST1VAFE3BX_REG_WAKE_UP_THS         = 0x1C,  // 唤醒阈值寄存器
    ST1VAFE3BX_REG_WAKE_UP_DUR         = 0x1D,  // 唤醒时长寄存器
    ST1VAFE3BX_REG_FREE_FALL           = 0x1E,  // 自由落体配置寄存器
    ST1VAFE3BX_REG_MD1_CFG             = 0x1F,  // 中断引脚路由配置寄存器

    // 中断状态寄存器
    ST1VAFE3BX_REG_WAKE_UP_SRC         = 0x21,  // 唤醒事件状态寄存器
    ST1VAFE3BX_REG_TAP_SRC             = 0x22,  // 轻击事件状态寄存器
    ST1VAFE3BX_REG_SIXD_SRC            = 0x23,  // 6D/4D事件状态寄存器
    ST1VAFE3BX_REG_ALL_INT_SRC         = 0x24,  // 所有中断汇总状态寄存器
    ST1VAFE3BX_REG_STATUS              = 0x25,  // 数据就绪状态寄存器

    // FIFO配置、状态与数据寄存器
    ST1VAFE3BX_REG_FIFO_CTRL           = 0x15,  // FIFO控制寄存器
    ST1VAFE3BX_REG_FIFO_WTM            = 0x16,  // FIFO-WTM配置寄存器
    ST1VAFE3BX_REG_FIFO_BATCH_DEC      = 0x47,  // FIFO批处理配置寄存器
    ST1VAFE3BX_REG_FIFO_STATUS1        = 0x26,  // FIFO状态寄存器1
    ST1VAFE3BX_REG_FIFO_STATUS2        = 0x27,  // FIFO状态寄存器2（未读样本数）
    ST1VAFE3BX_REG_FIFO_DATA_OUT_TAG   = 0x40,  // FIFO数据TAG寄存器
    ST1VAFE3BX_REG_FIFO_DATA_OUT_X_L   = 0x41,  // FIFO X轴低字节
    ST1VAFE3BX_REG_FIFO_DATA_OUT_X_H   = 0x42,  // FIFO X轴高字节
    ST1VAFE3BX_REG_FIFO_DATA_OUT_Y_L   = 0x43,  // FIFO Y轴低字节
    ST1VAFE3BX_REG_FIFO_DATA_OUT_Y_H   = 0x44,  // FIFO Y轴高字节
    ST1VAFE3BX_REG_FIFO_DATA_OUT_Z_L   = 0x45,  // FIFO Z轴低字节
    ST1VAFE3BX_REG_FIFO_DATA_OUT_Z_H   = 0x46,  // FIFO Z轴高字节


    //Accelerate、vAFE原始数据输出寄存器
    ST1VAFE3BX_REG_OUT_X_L             = 0x28,  // X轴加速度低字节
    ST1VAFE3BX_REG_OUT_X_H             = 0x29,  // X轴加速度高字节
    ST1VAFE3BX_REG_OUT_Y_L             = 0x2A,  // Y轴加速度低字节
    ST1VAFE3BX_REG_OUT_Y_H             = 0x2B,  // Y轴加速度高字节
    ST1VAFE3BX_REG_OUT_Z_L             = 0x2C,  // Z轴加速度低字节
    ST1VAFE3BX_REG_OUT_Z_H             = 0x2D,  // Z轴加速度高字节
    ST1VAFE3BX_REG_OUT_AH_BIO_L        = 0x2E,  // vAFE数据低字节
    ST1VAFE3BX_REG_OUT_AH_BIO_H        = 0x2F,  // vAFE数据高字节

    // vAFE/生物电通道寄存器
    ST1VAFE3BX_REG_AH_BIO_CFG1         = 0x30,  // vAFE配置寄存器1
    ST1VAFE3BX_REG_AH_BIO_CFG2         = 0x31,  // vAFE配置寄存器2（增益/阻抗/模式）
    ST1VAFE3BX_REG_AH_BIO_CFG3         = 0x32,  // vAFE配置寄存器3
    ST1VAFE3BX_REG_I3C_IF_CTRL         = 0x33,  // I3C接口控制寄存器

    // 嵌入式功能相关寄存器
    ST1VAFE3BX_REG_EMB_FUNC_STATUS_MAINPAGE  = 0x34,  // 嵌入式功能主状态寄存器
    ST1VAFE3BX_REG_FSM_STATUS_MAINPAGE       = 0x35,  // FSM主状态寄存器
    ST1VAFE3BX_REG_MLC_STATUS_MAINPAGE       = 0x36,  // MLC中断状态位

    // 设备配置与访问控制寄存器
    ST1VAFE3BX_REG_EN_DEVICE_CONFIG    = 0x3E,  // 设备配置使能寄存器
    ST1VAFE3BX_REG_FUNC_CFG_ACCESS     = 0x3F,  // 嵌入式功能寄存器访问控制寄存器

    // 轻击检测配置寄存器
    ST1VAFE3BX_REG_TAP_CFG0            = 0x6F,  // 轻击配置寄存器0
    ST1VAFE3BX_REG_TAP_CFG1            = 0x70,  // 轻击配置寄存器1
    ST1VAFE3BX_REG_TAP_CFG2            = 0x71,  // 轻击配置寄存器2
    ST1VAFE3BX_REG_TAP_CFG3            = 0x72,  // 轻击配置寄存器3
    ST1VAFE3BX_REG_TAP_CFG4            = 0x73,  // 轻击配置寄存器4
    ST1VAFE3BX_REG_TAP_CFG5            = 0x74,  // 轻击配置寄存器5
    ST1VAFE3BX_REG_TAP_CFG6            = 0x75,  // 轻击配置寄存器6

    // 时间戳寄存器
    ST1VAFE3BX_REG_TIMESTAMP0          = 0x7A,  // 时间戳字节0
    ST1VAFE3BX_REG_TIMESTAMP1          = 0x7B,  // 时间戳字节1
    ST1VAFE3BX_REG_TIMESTAMP2          = 0x7C,  // 时间戳字节2
    ST1VAFE3BX_REG_TIMESTAMP3          = 0x7D,  // 时间戳字节3

    //嵌入式功能页寄存器。需切换到embedded page访问
    ST1VAFE3BX_EMB_REG_MLC_INT          = 0x0D,  // MLC中断寄存器
    ST1VAFE3BX_EMB_REG_EMB_FUNC_FIFO_EN = 0x18,  // 嵌入式功能FIFO使能寄存器
    ST1VAFE3BX_EMB_REG_MLC1_SRC         = 0x34,  // MLC1结果寄存器（与MLC_STATUS_MAIN地址相同，但需切换访问）
    ST1VAFE3BX_EMB_REG_MLC2_SRC         = 0x35,  // MLC2结果寄存器（与FSM_STATUS_MAIN地址相同，但需切换访问）
    ST1VAFE3BX_EMB_REG_MLC3_SRC         = 0x36,  // MLC3结果寄存器（与MLC_STATUS_MAIN地址相同，但需切换访问）
    ST1VAFE3BX_EMB_REG_MLC4_SRC         = 0x37   // MLC4结果寄存器
} ST1VAFE3BX_REG;

/******************************************************************************
 * 4. FIFO 相关常量定义
 * 6.1 FIFO (page27-29)、8.42 FIFO_DATA_OUT_TAG (40h) (page56)
 *****************************************************************************/
/**
 * @brief FIFO 基础参数
 */
#define ST1VAFE3BX_FIFO_SAMPLE_LENGTH       (7)     // 单样本长度：1字节TAG + 6字节数据
#define ST1VAFE3BX_FIFO_MAX_DEPTH           (128)   // FIFO最大深度（标准模式）
#define ST1VAFE3BX_FIFO_2X_DEPTH            (256)   // FIFO 2倍深度模式（仅加速度计8位数据）
#define ST1VAFE3BX_FIFO_WATERMARK_MAX       (127)   // WTM最大值

/**
 * @brief FIFO TAG 定义（Table 107）
 * 用于识别FIFO中存储的数据类型
 */
typedef enum __ST1VAFE3BX_FIFO_TAG {
    ST1VAFE3BX_FIFO_TAG_EMPTY           = 0x00,  // FIFO空
    ST1VAFE3BX_FIFO_TAG_ACCEL_ONLY      = 0x02,  // 仅加速度计数据
    ST1VAFE3BX_FIFO_TAG_ACCEL_2X_DEPTH  = 0x03,  // 2倍深度模式加速度计数据
    ST1VAFE3BX_FIFO_TAG_TIMESTAMP       = 0x04,  // 时间戳/配置变更数据
    ST1VAFE3BX_FIFO_TAG_STEP_COUNTER    = 0x12,  // 计步器数据
    ST1VAFE3BX_FIFO_TAG_MLC_RESULT      = 0x1A,  // MLC结果数据
    ST1VAFE3BX_FIFO_TAG_MLC_FILTER      = 0x1B,  // MLC滤波器数据
    ST1VAFE3BX_FIFO_TAG_MLC_FEATURE     = 0x1C,  // MLC特征数据
    ST1VAFE3BX_FIFO_TAG_FSM_RESULT      = 0x1D,  // FSM结果数据
    ST1VAFE3BX_FIFO_TAG_VAFE_ONLY       = 0x1E,  // 仅vAFE/生物电数据
    ST1VAFE3BX_FIFO_TAG_ACCEL_VAFE      = 0x1F   // 加速度计+vAFE同步数据
} ST1VAFE3BX_FIFO_TAG;

/******************************************************************************
 * 5. 核心寄存器位偏移与掩码定义
 * 按寄存器分组，严格对应手册8.Register description章节
 *****************************************************************************/

/*-------------------------- EXT_CLK_CFG (08h) 寄存器 8.1节(page36) --------------------------*/
#define ST1VAFE3BX_EXT_CLK_CFG_EN_Pos            7
#define ST1VAFE3BX_EXT_CLK_CFG_EN_Msk            (0x01 << ST1VAFE3BX_EXT_CLK_CFG_EN_Pos)

/*-------------------------- PIN_CTRL (0Ch) 寄存器 8.2节(page36) --------------------------*/
#define ST1VAFE3BX_PIN_CTRL_SDO_PU_EN_Pos       7
#define ST1VAFE3BX_PIN_CTRL_SDO_PU_EN_Msk       (0x01 << ST1VAFE3BX_PIN_CTRL_SDO_PU_EN_Pos)
#define ST1VAFE3BX_PIN_CTRL_SDA_PU_EN_Pos       6
#define ST1VAFE3BX_PIN_CTRL_SDA_PU_EN_Msk       (0x01 << ST1VAFE3BX_PIN_CTRL_SDA_PU_EN_Pos)
#define ST1VAFE3BX_PIN_CTRL_H_LACTIVE_Pos       3
#define ST1VAFE3BX_PIN_CTRL_H_LACTIVE_Msk       (0x01 << ST1VAFE3BX_PIN_CTRL_H_LACTIVE_Pos)//中断输出极性
#define ST1VAFE3BX_PIN_CTRL_CS_PU_DIS_Pos       2
#define ST1VAFE3BX_PIN_CTRL_CS_PU_DIS_Msk       (0x01 << ST1VAFE3BX_PIN_CTRL_CS_PU_DIS_Pos)
#define ST1VAFE3BX_PIN_CTRL_PP_OD_Pos           1
#define ST1VAFE3BX_PIN_CTRL_PP_OD_Msk           (0x01 << ST1VAFE3BX_PIN_CTRL_PP_OD_Pos)
#define ST1VAFE3BX_PIN_CTRL_SIM_Pos             0
#define ST1VAFE3BX_PIN_CTRL_SIM_Msk             (0x01 << ST1VAFE3BX_PIN_CTRL_SIM_Pos)

/*-------------------------- CTRL1 (10h) 寄存器 8.5节(page37) --------------------------*/
#define ST1VAFE3BX_CTRL1_SMART_POWER_EN_Pos    7
#define ST1VAFE3BX_CTRL1_SMART_POWER_EN_Msk    (0x01 << ST1VAFE3BX_CTRL1_SMART_POWER_EN_Pos)
#define ST1VAFE3BX_CTRL1_INT_PIN_EN_Pos        6
#define ST1VAFE3BX_CTRL1_INT_PIN_EN_Msk        (0x01 << ST1VAFE3BX_CTRL1_INT_PIN_EN_Pos)
#define ST1VAFE3BX_CTRL1_SW_RESET_Pos          5
#define ST1VAFE3BX_CTRL1_SW_RESET_Msk          (0x01 << ST1VAFE3BX_CTRL1_SW_RESET_Pos)
#define ST1VAFE3BX_CTRL1_IF_ADD_INC_Pos        4
#define ST1VAFE3BX_CTRL1_IF_ADD_INC_Msk        (0x01 << ST1VAFE3BX_CTRL1_IF_ADD_INC_Pos)
#define ST1VAFE3BX_CTRL1_DRDY_PULSED_Pos       3
#define ST1VAFE3BX_CTRL1_DRDY_PULSED_Msk       (0x01 << ST1VAFE3BX_CTRL1_DRDY_PULSED_Pos)
#define ST1VAFE3BX_CTRL1_WU_X_EN_Pos           2
#define ST1VAFE3BX_CTRL1_WU_X_EN_Msk           (0x01 << ST1VAFE3BX_CTRL1_WU_X_EN_Pos)
#define ST1VAFE3BX_CTRL1_WU_Y_EN_Pos           1
#define ST1VAFE3BX_CTRL1_WU_Y_EN_Msk           (0x01 << ST1VAFE3BX_CTRL1_WU_Y_EN_Pos)
#define ST1VAFE3BX_CTRL1_WU_Z_EN_Pos           0
#define ST1VAFE3BX_CTRL1_WU_Z_EN_Msk           (0x01 << ST1VAFE3BX_CTRL1_WU_Z_EN_Pos)

/*-------------------------- CTRL2 (11h) 寄存器 8.6节(page38) --------------------------*/
#define ST1VAFE3BX_CTRL2_INT_BOOT_Pos           7
#define ST1VAFE3BX_CTRL2_INT_BOOT_Msk           (0x01 << ST1VAFE3BX_CTRL2_INT_BOOT_Pos)
#define ST1VAFE3BX_CTRL2_INT_FIFO_FULL_Pos      6
#define ST1VAFE3BX_CTRL2_INT_FIFO_FULL_Msk      (0x01 << ST1VAFE3BX_CTRL2_INT_FIFO_FULL_Pos)
#define ST1VAFE3BX_CTRL2_INT_FIFO_TH_Pos        5
#define ST1VAFE3BX_CTRL2_INT_FIFO_TH_Msk        (0x01 << ST1VAFE3BX_CTRL2_INT_FIFO_TH_Pos)
#define ST1VAFE3BX_CTRL2_INT_FIFO_OVR_Pos       4
#define ST1VAFE3BX_CTRL2_INT_FIFO_OVR_Msk       (0x01 << ST1VAFE3BX_CTRL2_INT_FIFO_OVR_Pos)
#define ST1VAFE3BX_CTRL2_INT_DRDY_Pos           3
#define ST1VAFE3BX_CTRL2_INT_DRDY_Msk           (0x01 << ST1VAFE3BX_CTRL2_INT_DRDY_Pos)

/*-------------------------- CTRL3 (12h) 寄存器 8.7节(page38) --------------------------*/
#define ST1VAFE3BX_CTRL3_HP_EN_Pos              2
#define ST1VAFE3BX_CTRL3_HP_EN_Msk              (0x01 << ST1VAFE3BX_CTRL3_HP_EN_Pos)
#define ST1VAFE3BX_CTRL3_ST_SIGN_Y_Pos          1
#define ST1VAFE3BX_CTRL3_ST_SIGN_Y_Msk          (0x01 << ST1VAFE3BX_CTRL3_ST_SIGN_Y_Pos)
#define ST1VAFE3BX_CTRL3_ST_SIGN_X_Pos          0
#define ST1VAFE3BX_CTRL3_ST_SIGN_X_Msk          (0x01 << ST1VAFE3BX_CTRL3_ST_SIGN_X_Pos)

/*-------------------------- CTRL4 (13h) 寄存器 8.8节(page39) --------------------------*/
#define ST1VAFE3BX_CTRL4_BDU_Pos                5
#define ST1VAFE3BX_CTRL4_BDU_Msk                (0x01 << ST1VAFE3BX_CTRL4_BDU_Pos)
#define ST1VAFE3BX_CTRL4_EMB_FUNC_EN_Pos        4
#define ST1VAFE3BX_CTRL4_EMB_FUNC_EN_Msk        (0x01 << ST1VAFE3BX_CTRL4_EMB_FUNC_EN_Pos)
#define ST1VAFE3BX_CTRL4_FIFO_EN_Pos            3
#define ST1VAFE3BX_CTRL4_FIFO_EN_Msk            (0x01 << ST1VAFE3BX_CTRL4_FIFO_EN_Pos)


/*-------------------------- CTRL5 (14h) 寄存器 8.9节(page40-41) --------------------------*/
// ODR配置（Table37）
#define ST1VAFE3BX_CTRL5_ODR_Pos                4
#define ST1VAFE3BX_CTRL5_ODR_Msk                (0x0F << ST1VAFE3BX_CTRL5_ODR_Pos)
typedef enum __ST1VAFE3BX_ODR {
    ST1VAFE3BX_ODR_POWER_DOWN               = (0x00 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_1_6_HZ_ULP               = (0x01 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_3_HZ_ULP                 = (0x02 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_25_HZ_ULP                = (0x03 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_6_HZ                     = (0x04 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_12_5_HZ                  = (0x05 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_25_HZ                    = (0x06 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_50_HZ                    = (0x07 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_100_HZ                   = (0x08 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_200_HZ                   = (0x09 << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_400_HZ                   = (0x0A << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_800_HZ                   = (0x0B << ST1VAFE3BX_CTRL5_ODR_Pos),
    ST1VAFE3BX_ODR_ONE_SHOT                 = (0x0F << ST1VAFE3BX_CTRL5_ODR_Pos)
} ST1VAFE3BX_ODR;

//带宽配置,high-performance模式下有效
#define ST1VAFE3BX_CTRL5_BW_Pos                 2
#define ST1VAFE3BX_CTRL5_BW_Msk                 (0x03 << ST1VAFE3BX_CTRL5_BW_Pos)
typedef enum __ST1VAFE3BX_BW {
    ST1VAFE3BX_BW_half                      = (0x00 << ST1VAFE3BX_CTRL5_BW_Pos),
    ST1VAFE3BX_BW_quarter                   = (0x01 << ST1VAFE3BX_CTRL5_BW_Pos),
    ST1VAFE3BX_BW_okta                      = (0x02 << ST1VAFE3BX_CTRL5_BW_Pos),
    ST1VAFE3BX_BW_sexdecimal                = (0x03 << ST1VAFE3BX_CTRL5_BW_Pos)
} ST1VAFE3BX_BW;

// 加速度计量程配置（Table40）
#define ST1VAFE3BX_CTRL5_FS_Pos                 0
#define ST1VAFE3BX_CTRL5_FS_Msk                 (0x03 << ST1VAFE3BX_CTRL5_FS_Pos)
typedef enum __ST1VAFE3BX_FS {
    ST1VAFE3BX_FS_2G                        = (0x00 << ST1VAFE3BX_CTRL5_FS_Pos),
    ST1VAFE3BX_FS_4G                        = (0x01 << ST1VAFE3BX_CTRL5_FS_Pos),
    ST1VAFE3BX_FS_8G                        = (0x02 << ST1VAFE3BX_CTRL5_FS_Pos),
    ST1VAFE3BX_FS_16G                       = (0x03 << ST1VAFE3BX_CTRL5_FS_Pos)
} ST1VAFE3BX_FS;

/*-------------------------- FIFO_CTRL (15h) 寄存器 8.10节(page42) --------------------------*/
//输出数据率 (ODR) 或 批处理数据率 (BDR) 发生修改时，控制是否把「配置变更信息 + 对应时间戳」存入FIFO
#define ST1VAFE3BX_FIFO_CTRL_CFG_CHG_EN_Pos    7
#define ST1VAFE3BX_FIFO_CTRL_CFG_CHG_EN_Msk    (0x01 << ST1VAFE3BX_FIFO_CTRL_CFG_CHG_EN_Pos)

//设置仅vafe下的双倍深度
#define ST1VAFE3BX_FIFO_CTRL_FIFO_DEPTH_Pos    6
#define ST1VAFE3BX_FIFO_CTRL_FIFO_DEPTH_Msk    (0x01 << ST1VAFE3BX_FIFO_CTRL_FIFO_DEPTH_Pos)

#define ST1VAFE3BX_FIFO_CTRL_FIFO_EN_ADV_Pos   4
#define ST1VAFE3BX_FIFO_CTRL_FIFO_EN_ADV_Msk   (0x01 << ST1VAFE3BX_FIFO_CTRL_FIFO_EN_ADV_Pos)

//FIFO深度限制为阈值
#define ST1VAFE3BX_FIFO_CTRL_STOP_ON_FTH_Pos   3
#define ST1VAFE3BX_FIFO_CTRL_STOP_ON_FTH_Msk   (0x01 << ST1VAFE3BX_FIFO_CTRL_STOP_ON_FTH_Pos)

//FIFO模式选择(3位)，对应Table42的FIFO_MODE[2:0]
#define ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Pos     0
#define ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Msk     (0x07 << ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Pos)
typedef enum __ST1VAFE3BX_FIFO_MODE {
    ST1VAFE3BX_FIFO_MODE_BYPASS            = (0x00 << ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Pos),  // 旁路模式
    ST1VAFE3BX_FIFO_MODE_FIFO              = (0x01 << ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Pos),  // FIFO模式
    ST1VAFE3BX_FIFO_MODE_CONTINUOUS        = (0x06 << ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Pos),  // 连续模式（满后覆盖旧数据）
    ST1VAFE3BX_FIFO_MODE_CONT_TO_FIFO      = (0x03 << ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Pos),  // 连续→FIFO模式
    ST1VAFE3BX_FIFO_MODE_BYPASS_TO_CONT    = (0x04 << ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Pos),  // 旁路→连续模式
    ST1VAFE3BX_FIFO_MODE_BYPASS_TO_FIFO    = (0x07 << ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Pos)   // 旁路→FIFO模式
} ST1VAFE3BX_FIFO_MODE;

/*-------------------------- FIFO_WTM (16h) 寄存器 8.11节(page43) --------------------------*/
//设置仅存储accel数据，default:0
#define ST1VAFE3BX_FIFO_WTM_XL_ONLY_FIFO_Pos   7
#define ST1VAFE3BX_FIFO_WTM_XL_ONLY_FIFO_Msk   (0x01 << ST1VAFE3BX_FIFO_WTM_XL_ONLY_FIFO_Pos)

//FIFO水印阈值(7位)：范围0~127，对应FTH[6:0]
#define ST1VAFE3BX_FIFO_WTM_FTH_Pos            0
#define ST1VAFE3BX_FIFO_WTM_FTH_Msk            (0x7F << ST1VAFE3BX_FIFO_WTM_FTH_Pos)

/*-------------------------- INTERRUPT_CFG (17h) 寄存器 8.12节 --------------------------*/
#define ST1VAFE3BX_INTERRUPT_CFG_TIMESTAMP_EN_Pos         7
#define ST1VAFE3BX_INTERRUPT_CFG_TIMESTAMP_EN_Msk         (0x01 << ST1VAFE3BX_INTERRUPT_CFG_TIMESTAMP_EN_Pos)
#define ST1VAFE3BX_INTERRUPT_CFG_WAKE_THS_W_Pos           5
#define ST1VAFE3BX_INTERRUPT_CFG_WAKE_THS_W_Msk           (0x01 << ST1VAFE3BX_INTERRUPT_CFG_WAKE_THS_W_Pos)
#define ST1VAFE3BX_INTERRUPT_CFG_SLEEP_STATUS_ON_INT_Pos  3
#define ST1VAFE3BX_INTERRUPT_CFG_SLEEP_STATUS_ON_INT_Msk  (0x01 << ST1VAFE3BX_INTERRUPT_CFG_SLEEP_STATUS_ON_INT_Pos)
#define ST1VAFE3BX_INTERRUPT_CFG_DIS_RST_LIR_ALL_INT_Pos  2
#define ST1VAFE3BX_INTERRUPT_CFG_DIS_RST_LIR_ALL_INT_Msk  (0x01 << ST1VAFE3BX_INTERRUPT_CFG_DIS_RST_LIR_ALL_INT_Pos)
#define ST1VAFE3BX_INTERRUPT_CFG_LIR_Pos                  1
#define ST1VAFE3BX_INTERRUPT_CFG_LIR_Msk                  (0x01 << ST1VAFE3BX_INTERRUPT_CFG_LIR_Pos)//1为锁存模式
#define ST1VAFE3BX_INTERRUPT_CFG_INTERRUPTS_ENABLE_Pos    0
#define ST1VAFE3BX_INTERRUPT_CFG_INTERRUPTS_ENABLE_Msk    (0x01 << ST1VAFE3BX_INTERRUPT_CFG_INTERRUPTS_ENABLE_Pos)

/*-------------------------- MD1_CFG (1Fh) 寄存器 8.16节 选择MLC等中断路由 --------------------------*/
#define ST1VAFE3BX_MD1_CFG_INT_EMB_FUNC_Pos               0
#define ST1VAFE3BX_MD1_CFG_INT_EMB_FUNC_Msk               (0x01 << ST1VAFE3BX_MD1_CFG_INT_EMB_FUNC_Pos)

/*------------------ ST1VAFE3BX_REG_FIFO_BATCH_DEC (47h) 寄存器 8.46节 ------------------*/
// 时间戳批处理速率配置
#define ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_Pos     3U
#define ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_Msk     (0x03UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_Pos)
typedef enum __ST1VAFE3BX_FIFO_BATCH_DEC_TS_BATCH {
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_NONE      = (0x00UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_1         = (0x01UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_8         = (0x02UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_32        = (0x03UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_Pos)
} ST1VAFE3BX_FIFO_BATCH_DEC_TS_BATCH;

//加速度计批处理数据率配置
#define ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos            0U
#define ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Msk            (0x07UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos)
typedef enum __ST1VAFE3BX_FIFO_BATCH_DEC_BDR_XL {
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_DEFAULT          = (0x00UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_2                = (0x01UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_4                = (0x02UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_8                = (0x03UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_16               = (0x04UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_32               = (0x05UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_64               = (0x06UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos),
    ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_NONE             = (0x07UL << ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Pos)
} ST1VAFE3BX_FIFO_BATCH_DEC_BDR_XL;

/*-------------------------- AH_BIO_CFG2 (31h) 寄存器 8.34节(page53) --------------------------*/

#define ST1VAFE3BX_AH_BIO_CFG2_AH_BIO_EN_Pos     0 //only_vafe使能位
#define ST1VAFE3BX_AH_BIO_CFG2_AH_BIO_EN_Msk     (0x01 << ST1VAFE3BX_AH_BIO_CFG2_AH_BIO_EN_Pos)
#define ST1VAFE3BX_AH_BIO_CFG2_GAIN_Pos          1
#define ST1VAFE3BX_AH_BIO_CFG2_GAIN_Msk          (0x03 << ST1VAFE3BX_AH_BIO_CFG2_GAIN_Pos)
typedef enum __ST1VAFE3BX_VAFE_GAIN {
    ST1VAFE3BX_VAFE_GAIN_2X                  = (0x00 << ST1VAFE3BX_AH_BIO_CFG2_GAIN_Pos),
    ST1VAFE3BX_VAFE_GAIN_4X                  = (0x01 << ST1VAFE3BX_AH_BIO_CFG2_GAIN_Pos),
    ST1VAFE3BX_VAFE_GAIN_8X                  = (0x02 << ST1VAFE3BX_AH_BIO_CFG2_GAIN_Pos),
    ST1VAFE3BX_VAFE_GAIN_16X                 = (0x03 << ST1VAFE3BX_AH_BIO_CFG2_GAIN_Pos)
} ST1VAFE3BX_VAFE_GAIN;
#define ST1VAFE3BX_AH_BIO_CFG2_ZIN_Pos           3
#define ST1VAFE3BX_AH_BIO_CFG2_ZIN_Msk           (0x03 << ST1VAFE3BX_AH_BIO_CFG2_ZIN_Pos)
typedef enum __ST1VAFE3BX_VAFE_ZIN {
    ST1VAFE3BX_VAFE_ZIN_100MOHM              = (0x00 << ST1VAFE3BX_AH_BIO_CFG2_ZIN_Pos),
    ST1VAFE3BX_VAFE_ZIN_200MOHM              = (0x01 << ST1VAFE3BX_AH_BIO_CFG2_ZIN_Pos),
    ST1VAFE3BX_VAFE_ZIN_500MOHM              = (0x02 << ST1VAFE3BX_AH_BIO_CFG2_ZIN_Pos),
    ST1VAFE3BX_VAFE_ZIN_1GOHM                = (0x03 << ST1VAFE3BX_AH_BIO_CFG2_ZIN_Pos)
} ST1VAFE3BX_VAFE_ZIN;
#define ST1VAFE3BX_AH_BIO_CFG2_MODE_Pos          5
#define ST1VAFE3BX_AH_BIO_CFG2_MODE_Msk          (0x03 << ST1VAFE3BX_AH_BIO_CFG2_MODE_Pos)
typedef enum __ST1VAFE3BX_VAFE_MODE {
    ST1VAFE3BX_VAFE_MODE_DIFFERENTIAL        = (0x00 << ST1VAFE3BX_AH_BIO_CFG2_MODE_Pos),
    ST1VAFE3BX_VAFE_MODE_SINGLE_ENDED_CH1    = (0x01 << ST1VAFE3BX_AH_BIO_CFG2_MODE_Pos),
    ST1VAFE3BX_VAFE_MODE_SINGLE_ENDED_CH2    = (0x02 << ST1VAFE3BX_AH_BIO_CFG2_MODE_Pos),
    ST1VAFE3BX_VAFE_MODE_FORCED_RESET        = (0x03 << ST1VAFE3BX_AH_BIO_CFG2_MODE_Pos)
} ST1VAFE3BX_VAFE_MODE;

/*-------------------------- MLC_STATUS_MAINPAGE (36h) 寄存器 8.39节(page55) --------------------------*/
#define ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC1_Pos 0
#define ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC1_Msk (0x01 << ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC1_Pos)
#define ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC2_Pos 1
#define ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC2_Msk (0x01 << ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC2_Pos)
#define ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC3_Pos 2
#define ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC3_Msk (0x01 << ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC3_Pos)
#define ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC4_Pos 3
#define ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC4_Msk (0x01 << ST1VAFE3BX_MLC_STATUS_MAINPAGE_IS_MLC4_Pos)

/*-------------------------- EN_DEVICE_CONFIG (3Eh) 寄存器 8.40节(page55) --------------------------*/
#define ST1VAFE3BX_EN_DEVICE_CONFIG_EN_Pos       0
#define ST1VAFE3BX_EN_DEVICE_CONFIG_EN_Msk       (0x01 << ST1VAFE3BX_EN_DEVICE_CONFIG_EN_Pos)

/*-------------------------- FUNC_CFG_ACCESS (3Fh) 寄存器 8.41节(page55) --------------------------*/
#define ST1VAFE3BX_FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS_Pos 7
#define ST1VAFE3BX_FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS_Msk (0x01 << ST1VAFE3BX_FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS_Pos)
#define ST1VAFE3BX_FUNC_CFG_ACCESS_MAIN_PAGE               (0x00U)
#define ST1VAFE3BX_FUNC_CFG_ACCESS_EMB_PAGE                ST1VAFE3BX_FUNC_CFG_ACCESS_EMB_FUNC_REG_ACCESS_Msk

/*-------------------------- MLC_INT (0Dh) 嵌入式寄存器 10.9节(page69) --------------------------*/
#define ST1VAFE3BX_MLC_INT_INT_MLC1_Pos                    0
#define ST1VAFE3BX_MLC_INT_INT_MLC1_Msk                    (0x01 << ST1VAFE3BX_MLC_INT_INT_MLC1_Pos)
#define ST1VAFE3BX_MLC_INT_INT_MLC2_Pos                    1
#define ST1VAFE3BX_MLC_INT_INT_MLC2_Msk                    (0x01 << ST1VAFE3BX_MLC_INT_INT_MLC2_Pos)
#define ST1VAFE3BX_MLC_INT_INT_MLC3_Pos                    2
#define ST1VAFE3BX_MLC_INT_INT_MLC3_Msk                    (0x01 << ST1VAFE3BX_MLC_INT_INT_MLC3_Pos)
#define ST1VAFE3BX_MLC_INT_INT_MLC4_Pos                    3
#define ST1VAFE3BX_MLC_INT_INT_MLC4_Msk                    (0x01 << ST1VAFE3BX_MLC_INT_INT_MLC4_Pos)

/*-------------------------- EMB_FUNC_FIFO_EN (18h) 嵌入式寄存器 10.14节(page71) --------------------------*/
#define ST1VAFE3BX_EMB_FUNC_FIFO_EN_STEP_COUNTER_FIFO_EN_Pos       0
#define ST1VAFE3BX_EMB_FUNC_FIFO_EN_STEP_COUNTER_FIFO_EN_Msk       (0x01 << ST1VAFE3BX_EMB_FUNC_FIFO_EN_STEP_COUNTER_FIFO_EN_Pos)
#define ST1VAFE3BX_EMB_FUNC_FIFO_EN_MLC_FIFO_EN_Pos                1
#define ST1VAFE3BX_EMB_FUNC_FIFO_EN_MLC_FIFO_EN_Msk                (0x01 << ST1VAFE3BX_EMB_FUNC_FIFO_EN_MLC_FIFO_EN_Pos)
#define ST1VAFE3BX_EMB_FUNC_FIFO_EN_MLC_FILTER_FEATURE_FIFO_EN_Pos 2
#define ST1VAFE3BX_EMB_FUNC_FIFO_EN_MLC_FILTER_FEATURE_FIFO_EN_Msk (0x01 << ST1VAFE3BX_EMB_FUNC_FIFO_EN_MLC_FILTER_FEATURE_FIFO_EN_Pos)
#define ST1VAFE3BX_EMB_FUNC_FIFO_EN_FSM_FIFO_EN_Pos                3
#define ST1VAFE3BX_EMB_FUNC_FIFO_EN_FSM_FIFO_EN_Msk                (0x01 << ST1VAFE3BX_EMB_FUNC_FIFO_EN_FSM_FIFO_EN_Pos)


/******************************************************************************
 * 6. 状态位掩码定义
 *****************************************************************************/
/**
 * @brief STATUS (25h) 寄存器 8.22节(page49)
 */
#define ST1VAFE3BX_STATUS_DATA_READY_MASK        (0x01)  // 数据就绪标志
#define ST1VAFE3BX_STATUS_INT_GLOBAL_MASK        (0x02)  // 全局中断标志

/**
 * @brief FIFO_STATUS1 (26h) 寄存器 8.23节(page50)
 */
#define ST1VAFE3BX_FIFO_STATUS1_WTM_MASK         (0x80)  // FIFO-WTM达到标志
#define ST1VAFE3BX_FIFO_STATUS1_OVR_MASK         (0x40)  // FIFO溢出标志

/**
 * @brief FIFO_STATUS2 (27h) 寄存器 8.24节(page50)
 */
#define ST1VAFE3BX_FIFO_STATUS2_FSS_MASK         (0xFF)  // FIFO未读样本数掩码（实测bit7有效）


#ifdef __cplusplus
}
#endif

#endif // __PERIPHERAL_INCLUDE_ST1VAFE3BX_REG_H__
