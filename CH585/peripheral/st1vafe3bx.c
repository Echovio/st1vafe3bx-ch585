#include "st1vafe3bx.h"
#include "mlc_DF.h"
#include "mlc_PF.h"
#include "mlc_INV.h"
#include "mlc_EV.h"
#include "ch58x_spi.h"
#include "ch58x_gpio.h"
#include "ch58x_clock.h"
#include "core_riscv_pfic.h"

/**
 * @file st1vafe3bx.c
 * @brief ST1VAFE3BX驱动核心实现（SPI适配、参数配置、配置应用与数据读取）
 */

/* ==================== 私有状态与通用工具 ==================== */
static bool st1vafe3bx_initialized = false;
static volatile bool st1vafe3bx_interrupt_pending = false;
static volatile bool st1vafe3bx_interrupt_active_high = false;
static uint32_t st1vafe3bx_pending_timestamp = 0U;
static bool st1vafe3bx_pending_timestamp_valid = false;

typedef struct _ST1VAFE3BX_MLC_PROFILE_CONFIG {
    ST1VAFE3BX_MLC_PROFILE_ID id;
    const struct mems_conf_op_list *confs;
    uint8_t conf_count;
    const struct mems_conf_output_list *output_lists;
    uint8_t output_count;
} ST1VAFE3BX_MLC_PROFILE_CONFIG;

static const ST1VAFE3BX_MLC_PROFILE_CONFIG st1vafe3bx_mlc_profiles[] = {
    {
        .id = ST1VAFE3BX_MLC_PROFILE_DF,
        .confs = mlc_df_confs,
        .conf_count = MLC_DF_SENSORS_NUM,
        .output_lists = mlc_df_output_lists,
        .output_count = MLC_DF_SENSORS_NUM
    },
    {
        .id = ST1VAFE3BX_MLC_PROFILE_PF,
        .confs = mlc_pf_confs,
        .conf_count = MLC_PF_SENSORS_NUM,
        .output_lists = mlc_pf_output_lists,
        .output_count = MLC_PF_SENSORS_NUM
    },
    {
        .id = ST1VAFE3BX_MLC_PROFILE_INV,
        .confs = mlc_inv_confs,
        .conf_count = MLC_INV_SENSORS_NUM,
        .output_lists = mlc_inv_output_lists,
        .output_count = MLC_INV_SENSORS_NUM
    },
    {
        .id = ST1VAFE3BX_MLC_PROFILE_EV,
        .confs = mlc_ev_confs,
        .conf_count = MLC_EV_SENSORS_NUM,
        .output_lists = mlc_ev_output_lists,
        .output_count = MLC_EV_SENSORS_NUM
    }
};

static const ST1VAFE3BX_MLC_PROFILE_CONFIG *st1vafe3bx_active_mlc_profile =
    &st1vafe3bx_mlc_profiles[0];

static bool st1vafe3bx_select_main_register_page(void);
static bool st1vafe3bx_select_embedded_register_page(void);
static bool st1vafe3bx_apply_mlc_op(const struct mems_conf_op *op);
static bool st1vafe3bx_config_accel_default(void);
static bool st1vafe3bx_config_vafe_default(void);
static bool st1vafe3bx_config_vafe_init(void);
static bool st1vafe3bx_enable_fifo_advanced_mode(void);
static bool st1vafe3bx_enable_mlc_feature_fifo_output(void);
static uint8_t st1vafe3bx_mlc_tree_id_from_reg_addr(uint8_t reg_addr);
static const ST1VAFE3BX_MLC_PROFILE_CONFIG *st1vafe3bx_find_mlc_profile(
    ST1VAFE3BX_MLC_PROFILE_ID profile_id);
static bool st1vafe3bx_decode_mlc_feature_fifo_sample(
    const ST1VAFE3BX_RAW_SAMPLE *raw_sample,
    ST1VAFE3BX_MLC_FEATURE_SAMPLE *feature);

/* 平台延时封装 */
static void st1vafe3bx_delay_us(uint32_t us) {
    while (us > 0) {
        uint16_t step = (us > 65535U) ? 65535U : (uint16_t)us;
        clock_delay_us(step);
        us -= step;
    }
}

static void st1vafe3bx_delay_ms(uint32_t ms) {
    st1vafe3bx_delay_us(ms * 1000U);
}

static const ST1VAFE3BX_MLC_PROFILE_CONFIG *st1vafe3bx_find_mlc_profile(
    ST1VAFE3BX_MLC_PROFILE_ID profile_id) {
    for (uint8_t i = 0U;
            i < MEMS_CONF_ARRAY_LEN(st1vafe3bx_mlc_profiles);
            i++) {
        if (st1vafe3bx_mlc_profiles[i].id == profile_id) {
            return &st1vafe3bx_mlc_profiles[i];
        }
    }

    return NULL;
}

bool st1vafe3bx_set_mlc_profile(ST1VAFE3BX_MLC_PROFILE_ID profile_id) {
    const ST1VAFE3BX_MLC_PROFILE_CONFIG *profile =
        st1vafe3bx_find_mlc_profile(profile_id);

    if (profile == NULL) {
        return false;
    }

    st1vafe3bx_active_mlc_profile = profile;
    return true;
}

ST1VAFE3BX_MLC_PROFILE_ID st1vafe3bx_get_mlc_profile(void) {
    return st1vafe3bx_active_mlc_profile->id;
}

bool st1vafe3bx_reload_mlc_profile(ST1VAFE3BX_MLC_PROFILE_ID profile_id) {
    st1vafe3bx_stop();

    if (!st1vafe3bx_set_mlc_profile(profile_id)) {
        return false;
    }

    if (!st1vafe3bx_apply_mlc_config()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_config_mlc_interrupt()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_start()) {
        st1vafe3bx_stop();
        return false;
    }

    st1vafe3bx_clear_interrupt_pending();
    return true;
}

/* ==================== 模式启动入口 ==================== */

/* FIFO训练数据采集模式启动入口：SPI初始化 -> 芯片初始化 -> 默认配置 -> 开始采样 */
bool st1vafe3bx_startup_fifo_mode(void) {
    if (!st1vafe3bx_spi0_init()) {
        return false;
    }

    if (!st1vafe3bx_init()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_apply_default_config()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_start()) {
        st1vafe3bx_stop();
        return false;
    }

    return true;
}

/* MLC分类模式启动入口：SPI初始化 -> 芯片初始化 -> vAFE/XL配置 -> MLC配置 -> 中断路由 -> 开始采样 */
bool st1vafe3bx_startup_mlc_mode(void) {
    if (!st1vafe3bx_spi0_init()) {
        return false;
    }

    if (!st1vafe3bx_init()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_config_accel_default()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_config_vafe_init()) {
        st1vafe3bx_stop();
        return false;
    }
    st1vafe3bx_delay_ms(1000);

    if (!st1vafe3bx_config_vafe_default()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_apply_mlc_config()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_config_mlc_interrupt()) {
        st1vafe3bx_stop();
        return false;
    }

    if (!st1vafe3bx_start()) {
        st1vafe3bx_stop();
        return false;
    }

    return true;
}

/* ==================== 数据拼接与格式调整 ==================== */

/* 小端拼接两字节为有符号16位 低字节在前，高字节在后，12位ADC */
static bool st1vafe3bx_bytes_to_int16(uint8_t low, uint8_t high, int16_t *value) {
    if (value == NULL) {
        return false;
    }
    *value = (int16_t)(((uint16_t)high << 8) | low);
    return true;
}

/* 12位右对齐数据调整为int16_t左对齐格式 */
static int16_t st1vafe3bx_align_12bit_left(uint16_t value12) {
    return (int16_t)((value12 & 0x0FFFU) << 4);
}

/* ==================== SPI总线平台适配接口实现 ==================== */
/* SPI0初始化 */
bool st1vafe3bx_spi0_init(void) {
    spi_0_master_init(
        ST1VAFE3BX_SPI0_PORT,
        ST1VAFE3BX_SPI0_CLOCK_DIV,
        ST1VAFE3BX_SPI0_MODE, /* 0模式，空闲CLK为低电平，数据在CLK上升沿采样 */
        ST1VAFE3BX_SPI0_ORDER);
    return true;
}

/* CS片选PA12启动 */
static bool st1vafe3bx_spi0_chip_select(bool active) {
    return spi_0_set_channel_state(ST1VAFE3BX_SPI0_CS_CHANNEL, active);
}

/* 读取PA1中断脚电平作为DRDY输入 */
static bool st1vafe3bx_spi0_drdy_pin(bool *level) {
    if (level == NULL) {
        return false;
    }
    *level = ((R32_PA_PIN & (uint32_t)ST1VAFE3BX_CH585_INT_PIN) != 0U);
    return true;
}

/* 写入命令，length指示写入字节数，不包含命令字节 */
static bool st1vafe3bx_spi0_bus_write(
    uint8_t reg_addr,
    const uint8_t *write_data,
    uint16_t write_length) {
    if (write_data == NULL || write_length == 0 || reg_addr > ST1VAFE3BX_REG_MAX_ADDR) {
        return false;
    }

    if (!st1vafe3bx_spi0_chip_select(true)) {
        return false;
    }

    uint8_t cmd = ST1VAFE3BX_SPI_BUILD_CMD(reg_addr, ST1VAFE3BX_SPI_WRITE_FLAG);
    if (!spi_0_master_send_byte(cmd)) {
        st1vafe3bx_spi0_chip_select(false);
        return false;
    }

    for (uint16_t i = 0; i < write_length; i++) {
        if (!spi_0_master_send_byte(write_data[i])) {
            st1vafe3bx_spi0_chip_select(false);
            return false;
        }
    }

    if (!st1vafe3bx_spi0_chip_select(false)) {
        return false;
    }

    return true;
}

/* 读取命令，length指示读取字节数，不包含命令字节 */
static bool st1vafe3bx_spi0_bus_read(
    uint8_t reg_addr,
    uint8_t *read_data,
    uint16_t read_length) {
    if (read_data == NULL || read_length == 0 || reg_addr > ST1VAFE3BX_REG_MAX_ADDR) {
        return false;
    }

    if (!st1vafe3bx_spi0_chip_select(true)) {
        return false;
    }

    uint8_t cmd = ST1VAFE3BX_SPI_BUILD_CMD(reg_addr, ST1VAFE3BX_SPI_READ_FLAG);
    if (!spi_0_master_send_byte(cmd)) {
        st1vafe3bx_spi0_chip_select(false);
        return false;
    }

    /* 其余字节：调用spi_0_master_receive_byte发送0xFF进行数据交换 */
    for (uint16_t i = 0; i < read_length; i++) {
        if (!spi_0_master_receive_byte(&read_data[i])) {
            st1vafe3bx_spi0_chip_select(false);
            return false;
        }
    }

    if (!st1vafe3bx_spi0_chip_select(false)) {
        return false;
    }

    return true;
}

/* DMA读取命令，length指示读取字节数，不包含命令字节 */
static bool st1vafe3bx_spi0_bus_read_dma(
    uint8_t reg_addr,
    uint8_t *read_data,
    uint16_t read_length) {
    if (read_data == NULL || read_length == 0U || reg_addr > ST1VAFE3BX_REG_MAX_ADDR) {
        return false;
    }
    if (read_length > 4095U) {
        return false;
    }

    if (!st1vafe3bx_spi0_chip_select(true)) {
        return false;
    }

    uint8_t cmd = ST1VAFE3BX_SPI_BUILD_CMD(reg_addr, ST1VAFE3BX_SPI_READ_FLAG);
    if (!spi_0_master_send_byte(cmd)) {
        st1vafe3bx_spi0_chip_select(false);
        return false;
    }

    if (!spi_0_master_receive_dma_bytes(read_data, read_length, true)) {
        st1vafe3bx_spi0_chip_select(false);
        return false;
    }

    if (!st1vafe3bx_spi0_chip_select(false)) {
        return false;
    }

    return true;
}

/* ==================== 基础寄存器访问接口封装 ==================== */

/**
 * @brief 读取单寄存器
 */
bool st1vafe3bx_read_register(ST1VAFE3BX_REG reg, uint8_t *value) {
    if (value == NULL) {
        return false;
    }
    return st1vafe3bx_read_registers(reg, value, 1);
}

/**
 * @brief 连续读寄存器
 * @param start_reg 起始寄存器地址
 * @param data 读取数据缓冲区
 * @param length 读取字节数
 */
bool st1vafe3bx_read_registers(
    ST1VAFE3BX_REG start_reg,
    uint8_t *data,
    uint8_t length) {
    if (data == NULL || length == 0) {
        return false;
    }
    if (!st1vafe3bx_initialized) {
        return false;
    }
    return st1vafe3bx_spi0_bus_read((uint8_t)start_reg, data, length);
}

bool st1vafe3bx_read_register_dma(ST1VAFE3BX_REG reg, uint8_t *value) {
    if (value == NULL) {
        return false;
    }
    return st1vafe3bx_read_registers_dma(reg, value, 1);
}

bool st1vafe3bx_read_registers_dma(
    ST1VAFE3BX_REG start_reg,
    uint8_t *data,
    uint16_t length) {
    if (data == NULL || length == 0U) {
        return false;
    }
    if (!st1vafe3bx_initialized) {
        return false;
    }
    return st1vafe3bx_spi0_bus_read_dma((uint8_t)start_reg, data, length);
}

/**
 * @brief 写入单寄存器。
 */
bool st1vafe3bx_write_register(ST1VAFE3BX_REG reg, uint8_t value) {
    return st1vafe3bx_write_registers(reg, &value, 1);
}

/**
 * @brief 读取 WHO_AM_I
 */
bool st1vafe3bx_read_who_am_i(uint8_t *who_am_i) {
    return st1vafe3bx_read_register(ST1VAFE3BX_REG_WHO_AM_I, who_am_i);
}

/* 查询驱动初始化状态 */
bool st1vafe3bx_is_initialized(void) {
    return st1vafe3bx_initialized;
}

/**
 * @brief 连续写寄存器
 * @param start_reg 起始寄存器地址
 * @param data 待写入数据缓冲区
 * @param length 写入字节数
 */
bool st1vafe3bx_write_registers(
    ST1VAFE3BX_REG start_reg,
    const uint8_t *data,
    uint8_t length) {
    if (data == NULL || length == 0) {
        return false;
    }
    if (!st1vafe3bx_initialized) {
        return false;
    }
    return st1vafe3bx_spi0_bus_write((uint8_t)start_reg, data, length);
}

/**
 * @brief 切回主寄存器页，恢复普通控制/状态寄存器访问。
 */
static bool st1vafe3bx_select_main_register_page(void) {
    return st1vafe3bx_write_register(
        ST1VAFE3BX_REG_FUNC_CFG_ACCESS,
        ST1VAFE3BX_FUNC_CFG_ACCESS_MAIN_PAGE);
}

/**
 * @brief 切到嵌入式功能寄存器页，用于访问 MLC/FSM 等功能寄存器。
 */
static bool st1vafe3bx_select_embedded_register_page(void) {
    return st1vafe3bx_write_register(
        ST1VAFE3BX_REG_FUNC_CFG_ACCESS,
        ST1VAFE3BX_FUNC_CFG_ACCESS_EMB_PAGE);
}

/* ==================== MLC配置、路由与结果读取 ==================== */

/**
 * @brief 执行 MEMS Studio 导出的单条 MLC 配置操作，MLC.h中自动实现寄存器页切换
 */
#define ST1VAFE3BX_MLC_POLL_TIMEOUT_MS        1000U
#define ST1VAFE3BX_MLC_TREE_MASK_ALL          (0x0FU)

static uint8_t st1vafe3bx_mlc_tree_id_from_reg_addr(uint8_t reg_addr) {
    if (reg_addr < (uint8_t)ST1VAFE3BX_EMB_REG_MLC1_SRC ||
            reg_addr > (uint8_t)ST1VAFE3BX_EMB_REG_MLC4_SRC) {
        return 0U;
    }

    return (uint8_t)(reg_addr - (uint8_t)ST1VAFE3BX_EMB_REG_MLC1_SRC + 1U);
}

static bool st1vafe3bx_apply_mlc_op(const struct mems_conf_op *op) {
    if (op == NULL) {
        return false;
    }

    switch (op->type) {
        case MEMS_CONF_OP_TYPE_READ: {
            uint8_t value = 0U;
            return st1vafe3bx_read_register((ST1VAFE3BX_REG)op->address, &value);
        }
        case MEMS_CONF_OP_TYPE_WRITE:
            return st1vafe3bx_write_register((ST1VAFE3BX_REG)op->address, op->data);
        case MEMS_CONF_OP_TYPE_DELAY:
            st1vafe3bx_delay_ms(op->data);
            return true;
        case MEMS_CONF_OP_TYPE_POLL_SET:
        case MEMS_CONF_OP_TYPE_POLL_RESET: {
            uint8_t value = 0U;
            for (uint16_t i = 0U; i < ST1VAFE3BX_MLC_POLL_TIMEOUT_MS; i++) {
                if (!st1vafe3bx_read_register((ST1VAFE3BX_REG)op->address, &value)) {
                    return false;
                }

                if (op->type == MEMS_CONF_OP_TYPE_POLL_SET) {
                    if ((value & op->data) == op->data) {
                        return true;
                    }
                } else {
                    if ((value & op->data) == 0U) {
                        return true;
                    }
                }
                st1vafe3bx_delay_ms(1U);
            }
            return false;
        }
        default:
            return false;
    }
}

bool st1vafe3bx_apply_mlc_config(void) {
    const ST1VAFE3BX_MLC_PROFILE_CONFIG *profile = st1vafe3bx_active_mlc_profile;

    if (profile == NULL || profile->conf_count == 0U || profile->confs == NULL) {
        return false;
    }

    for (uint8_t list_index = 0U; list_index < profile->conf_count; list_index++) {
        const struct mems_conf_op_list *op_list = &profile->confs[list_index];

        if (op_list->list == NULL || op_list->len == 0U) {
            return false;
        }

        for (uint32_t i = 0U; i < op_list->len; i++) {
            if (!st1vafe3bx_apply_mlc_op(&op_list->list[i])) {
                st1vafe3bx_select_main_register_page();
                return false;
            }
        }
    }

    return st1vafe3bx_select_main_register_page();
}

uint8_t st1vafe3bx_get_mlc_tree_mask(void) {
    const ST1VAFE3BX_MLC_PROFILE_CONFIG *profile = st1vafe3bx_active_mlc_profile;
    uint8_t tree_mask = 0U;

    if (profile == NULL || profile->output_count == 0U || profile->output_lists == NULL) {
        return 0U;
    }

    /*
     * MLC动作配置由MEMS Studio导出，output metadata中的reg_addr指向MLCx_SRC。
     * 以metadata作为唯一事实来源，可自动适配单树/双树/多树配置。
     */
    for (uint8_t list_index = 0U; list_index < profile->output_count; list_index++) {
        const struct mems_conf_output_list *output_list =
            &profile->output_lists[list_index];

        if (output_list->list == NULL) {
            continue;
        }

        for (uint16_t i = 0U; i < output_list->len; i++) {
            const struct mems_conf_output *output = &output_list->list[i];
            uint8_t tree_id = 0U;

            if (output->core != MEMS_CONF_OUTPUT_CORE_MLC) {
                continue;
            }

            tree_id = st1vafe3bx_mlc_tree_id_from_reg_addr(output->reg_addr);
            if (tree_id == 0U) {
                continue;
            }

            tree_mask |= (uint8_t)(1U << (tree_id - 1U));
        }
    }

    return (uint8_t)(tree_mask & ST1VAFE3BX_MLC_TREE_MASK_ALL);
}

// 配置MLC中断输出到INT1引脚，默认配置为非锁存、非时间戳、基本INT引脚使能、FIFO阈值中断禁用、高电平有效、推挽输出
bool st1vafe3bx_config_mlc_interrupt(void) {
    ST1VAFE3BX_INTERRUPT_CONFIG interrupt_config = {
        .int_pin_enable = true,
        .timestamp_enable = false,
        .basic_int_pin_enable = false,
        .int_fifo_threshold_enable = false,
        .level_interrupt_lir = false,
        .interrupt_polarity_active_low = false,
        .od_output = false
    };
    uint8_t mlc_int = 0U;
    uint8_t md1_cfg = 0U;
    uint8_t tree_mask = st1vafe3bx_get_mlc_tree_mask();

    if (tree_mask == 0U) {
        return false;
    }

    if (!st1vafe3bx_select_embedded_register_page()) {
        return false;
    }

    if (!st1vafe3bx_read_register(ST1VAFE3BX_EMB_REG_MLC_INT, &mlc_int)) {
        st1vafe3bx_select_main_register_page();
        return false;
    }
    mlc_int &= (uint8_t)(~ST1VAFE3BX_MLC_TREE_MASK_ALL);
    mlc_int |= (uint8_t)(tree_mask & ST1VAFE3BX_MLC_TREE_MASK_ALL);
    if (!st1vafe3bx_write_register(ST1VAFE3BX_EMB_REG_MLC_INT, mlc_int)) {
        st1vafe3bx_select_main_register_page();
        return false;
    }

    if (!st1vafe3bx_select_main_register_page()) {
        return false;
    }

    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_MD1_CFG, &md1_cfg)) {
        return false;
    }
    md1_cfg |= ST1VAFE3BX_MD1_CFG_INT_EMB_FUNC_Msk;
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_MD1_CFG, md1_cfg)) {
        return false;
    }

    return st1vafe3bx_config_interrupt(&interrupt_config);
}

bool st1vafe3bx_read_mlc_status_mask(uint8_t *status_mask) {
    uint8_t status = 0U;

    if (status_mask == NULL) {
        return false;
    }

    if (!st1vafe3bx_select_main_register_page()) {
        return false;
    }

    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_MLC_STATUS_MAINPAGE, &status)) {
        return false;
    }

    *status_mask = (uint8_t)(status & ST1VAFE3BX_MLC_TREE_MASK_ALL);
    return true;
}

bool st1vafe3bx_read_mlc_result(uint8_t tree_id, uint8_t *result) {
    ST1VAFE3BX_REG result_reg = ST1VAFE3BX_EMB_REG_MLC1_SRC;

    if (result == NULL || tree_id == 0U || tree_id > ST1VAFE3BX_MLC_TREE_MAX) {
        return false;
    }

    if (!st1vafe3bx_select_embedded_register_page()) {
        return false;
    }

    result_reg = (ST1VAFE3BX_REG)((uint8_t)ST1VAFE3BX_EMB_REG_MLC1_SRC + tree_id - 1U);
    if (!st1vafe3bx_read_register(result_reg, result)) {
        st1vafe3bx_select_main_register_page();
        return false;
    }

    return st1vafe3bx_select_main_register_page();
}

bool st1vafe3bx_read_mlc_results(uint8_t tree_mask, uint8_t *results) {
    if (results == NULL || (tree_mask & ST1VAFE3BX_MLC_TREE_MASK_ALL) == 0U) {
        return false;
    }

    if (!st1vafe3bx_select_embedded_register_page()) {
        return false;
    }

    /*
     * 双树同时变化时保持在embedded page内连续读取，避免每棵树单独
     * 切页导致偶发读到无效值。
     */
    for (uint8_t tree_id = 1U; tree_id <= ST1VAFE3BX_MLC_TREE_MAX; tree_id++) {
        uint8_t tree_bit = (uint8_t)(1U << (tree_id - 1U));
        ST1VAFE3BX_REG result_reg =
            (ST1VAFE3BX_REG)((uint8_t)ST1VAFE3BX_EMB_REG_MLC1_SRC + tree_id - 1U);

        if ((tree_mask & tree_bit) == 0U) {
            continue;
        }

        if (!st1vafe3bx_read_register(result_reg, &results[tree_id - 1U])) {
            st1vafe3bx_select_main_register_page();
            return false;
        }
    }

    return st1vafe3bx_select_main_register_page();
}

static bool st1vafe3bx_mlc_results_are_valid(uint8_t tree_mask, const uint8_t *results) {
    if (results == NULL) {
        return false;
    }

    for (uint8_t tree_id = 1U; tree_id <= ST1VAFE3BX_MLC_TREE_MAX; tree_id++) {
        uint8_t tree_bit = (uint8_t)(1U << (tree_id - 1U));

        if ((tree_mask & tree_bit) == 0U) {
            continue;
        }

        if (!st1vafe3bx_is_mlc_result_valid(tree_id, results[tree_id - 1U])) {
            return false;
        }
    }

    return true;
}

bool st1vafe3bx_read_valid_mlc_results(uint8_t tree_mask, uint8_t *results) {
    while (true) {
        if (!st1vafe3bx_read_mlc_results(tree_mask, results)) {
            return false;
        }
        if (st1vafe3bx_mlc_results_are_valid(tree_mask, results)) {
            return true;
        }

        /* MLCx_SRC偶发空读常见为0xFF，等待下一个稳定读数。 */
        st1vafe3bx_delay_us(10U);
    }
}

bool st1vafe3bx_is_mlc_result_valid(uint8_t tree_id, uint8_t result) {
    const ST1VAFE3BX_MLC_PROFILE_CONFIG *profile = st1vafe3bx_active_mlc_profile;

    if (profile == NULL || profile->output_count == 0U || profile->output_lists == NULL) {
        return false;
    }

    for (uint8_t list_index = 0U; list_index < profile->output_count; list_index++) {
        const struct mems_conf_output_list *output_list =
            &profile->output_lists[list_index];

        if (output_list->list == NULL) {
            continue;
        }

        for (uint16_t i = 0U; i < output_list->len; i++) {
            const struct mems_conf_output *output = &output_list->list[i];

            if (st1vafe3bx_mlc_tree_id_from_reg_addr(output->reg_addr) != tree_id) {
                continue;
            }
            if (output->results == NULL) {
                return false;
            }

            for (uint8_t j = 0U; j < output->num_results; j++) {
                if (output->results[j].code == result) {
                    return true;
                }
            }
        }
    }

    return false;
}

/* ==================== MLC feature FIFO调试接口 ==================== */

/**
 * @brief 打开FIFO advanced路径，使嵌入式功能结果可以进入FIFO。
 */
static bool st1vafe3bx_enable_fifo_advanced_mode(void) {
    uint8_t fifo_ctrl = 0U;

    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_FIFO_CTRL, &fifo_ctrl)) {
        return false;
    }

    fifo_ctrl |= ST1VAFE3BX_FIFO_CTRL_FIFO_EN_ADV_Msk;
    return st1vafe3bx_write_register(ST1VAFE3BX_REG_FIFO_CTRL, fifo_ctrl);
}

/**
 * @brief 在embedded page打开MLC feature写入FIFO开关。
 * @note EMB_FUNC_FIFO_EN(18h).MLC_FILTER_FEATURE_FIFO_EN=1后，MLC feature包以TAG=1Ch进入FIFO。
 */
static bool st1vafe3bx_enable_mlc_feature_fifo_output(void) {
    uint8_t emb_func_fifo_en = 0U;

    if (!st1vafe3bx_select_embedded_register_page()) {
        return false;
    }
    if (!st1vafe3bx_read_register(ST1VAFE3BX_EMB_REG_EMB_FUNC_FIFO_EN, &emb_func_fifo_en)) {
        st1vafe3bx_select_main_register_page();
        return false;
    }
    emb_func_fifo_en |= ST1VAFE3BX_EMB_FUNC_FIFO_EN_MLC_FILTER_FEATURE_FIFO_EN_Msk;
    if (!st1vafe3bx_write_register(ST1VAFE3BX_EMB_REG_EMB_FUNC_FIFO_EN, emb_func_fifo_en)) {
        st1vafe3bx_select_main_register_page();
        return false;
    }
    return st1vafe3bx_select_main_register_page();
}

/**
 * @brief 启动MLC feature相关FIFO配置
 */
bool st1vafe3bx_start_mlc_feature_fifo(
    const ST1VAFE3BX_FIFO_CONFIG *fifo_config) {
    ST1VAFE3BX_FIFO_CONFIG fifo_config_local = {0};

    if (fifo_config == NULL) {
        return false;
    }

    fifo_config_local = *fifo_config;
    if (!st1vafe3bx_config_fifo(&fifo_config_local)) {
        return false;
    }

    if (!st1vafe3bx_enable_fifo_advanced_mode()) {
        return false;
    }

    if (!st1vafe3bx_enable_mlc_feature_fifo_output()) {
        return false;
    }

    return true;
}

/**
 * @brief 判断FIFO原始样本是否为MLC feature包，并拼接原始字段。
 */
static bool st1vafe3bx_decode_mlc_feature_fifo_sample(
    const ST1VAFE3BX_RAW_SAMPLE *raw_sample,
    ST1VAFE3BX_MLC_FEATURE_SAMPLE *feature) {
    uint8_t tag_id_h5 = 0U;

    if (raw_sample == NULL || feature == NULL) {
        return false;
    }

    tag_id_h5 = (uint8_t)(raw_sample->tag >> 3);
    if (tag_id_h5 != ST1VAFE3BX_FIFO_TAG_MLC_FEATURE) {
        return false;
    }

    feature->raw_tag = raw_sample->tag;
    feature->value_bits = (uint16_t)(((uint16_t)raw_sample->data[1] << 8) | raw_sample->data[0]);
    feature->feature_id = (uint16_t)(((uint16_t)raw_sample->data[3] << 8) | raw_sample->data[2]);
    feature->reserved = (uint16_t)(((uint16_t)raw_sample->data[5] << 8) | raw_sample->data[4]);
    return true;
}

/**
 * @brief 主动读取FIFO中的MLC feature包，仅做字段拼接，不做数值解算。
 */
__HIGH_CODE
bool st1vafe3bx_read_mlc_feature_fifo(
    ST1VAFE3BX_MLC_FEATURE_SAMPLE *features,
    uint16_t feature_capacity,
    uint16_t *feature_count,
    uint16_t *raw_sample_count) {
    bool overflow = false;

    if (feature_count == NULL) {
        return false;
    }
    if (feature_capacity > 0U && features == NULL) {
        return false;
    }

    *feature_count = 0U;
    if (raw_sample_count != NULL) {
        *raw_sample_count = 0U;
    }

    while (true) {
        uint8_t fifo_count = 0U;

        if (!st1vafe3bx_get_fifo_count(&fifo_count)) {
            return false;
        }
        if (fifo_count == 0U) {
            break;
        }

        for (uint8_t i = 0U; i < fifo_count; i++) {
            ST1VAFE3BX_RAW_SAMPLE raw_sample = {0};
            ST1VAFE3BX_MLC_FEATURE_SAMPLE feature = {0};

            if (!st1vafe3bx_read_fifo_sample_dma(&raw_sample)) {
                return false;
            }
            if (raw_sample_count != NULL) {
                (*raw_sample_count)++;
            }

            if (!st1vafe3bx_decode_mlc_feature_fifo_sample(&raw_sample, &feature)) {
                continue;
            }

            if (*feature_count < feature_capacity) {
                features[*feature_count] = feature;
                (*feature_count)++;
            } else {
                overflow = true;
            }
        }
    }

    st1vafe3bx_clear_interrupt_pending();
    return !overflow;
}

/* ==================== 芯片生命周期控制 ==================== */

/**
 * @brief 初始化驱动并绑定平台接口，芯片上电与身份校验
 * @param interface 平台硬件接口（SPI、片选、延时、DRDY引脚等）
 * @return 初始化成功返回true，失败返回false
 */
bool st1vafe3bx_init(void) {
    st1vafe3bx_initialized = true;

    if (!st1vafe3bx_power_up_cmd()) {
        st1vafe3bx_initialized = false;
        return false;
    }
    if (!st1vafe3bx_soft_reset()) {
        st1vafe3bx_initialized = false;
        return false;
    }
    st1vafe3bx_delay_ms(5);

    return true;
}

/**
 * @brief 上电命令（强制回主寄存器页、使能配置访问并校验WHO_AM_I）
 * @note MCU复位但传感器未掉电时，器件可能仍停留在embedded page；
 *       先写FUNC_CFG_ACCESS=0确保后续EN_DEVICE_CONFIG/WHO_AM_I访问主寄存器页。
 */
bool st1vafe3bx_power_up_cmd(void) {
    uint8_t en_device_config = ST1VAFE3BX_EN_DEVICE_CONFIG_EN_Msk;
    uint8_t who_am_i = 0;

    if (!st1vafe3bx_select_main_register_page()) {
        return false;
    }

    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_EN_DEVICE_CONFIG, en_device_config)) {
        return false;
    }

    st1vafe3bx_delay_ms(25);

    if (!st1vafe3bx_read_who_am_i(&who_am_i)) {
        return false;
    }
    return (who_am_i == ST1VAFE3BX_WHO_AM_I_FIXED_VALUE);
}

/**
 * @brief 软件复位 CTRL1寄存器SW_RESET位
 */
bool st1vafe3bx_soft_reset(void) {
    uint8_t ctrl1_val = 0;
    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_CTRL1, &ctrl1_val)) {
        return false;
    }
    ctrl1_val |= ST1VAFE3BX_CTRL1_SW_RESET_Msk;
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_CTRL1, ctrl1_val)) {
        return false;
    }
    st1vafe3bx_delay_ms(5);
    return true;
}

/**
 * @brief 启动采样（配置CTRL5寄存器ODR位，默认800Hz高性能模式）
 * @param  输出数据率配置，CTRL5寄存器ODR[3:0]位值，寄存器[7:4]位，启动高性能模式
 */
bool st1vafe3bx_start(void) {
    uint8_t ctrl5_val = 0;
    uint8_t ctrl3_val = 0;
    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_CTRL5, &ctrl5_val)) {
        return false;
    }
    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_CTRL3, &ctrl3_val)) {
        return false;
    }
    ctrl5_val &= (uint8_t)(~ST1VAFE3BX_CTRL5_ODR_Msk);
    ctrl5_val |= ST1VAFE3BX_ODR_800_HZ;
    ctrl3_val &= (uint8_t)(~ST1VAFE3BX_CTRL3_HP_EN_Msk);
    ctrl3_val |= ST1VAFE3BX_CTRL3_HP_EN_Msk;
    return (
        st1vafe3bx_write_register(ST1VAFE3BX_REG_CTRL5, ctrl5_val) &&
        st1vafe3bx_write_register(ST1VAFE3BX_REG_CTRL3, ctrl3_val)
    );
}

/**
 * @brief 停止采样（配置CTRL5寄存器ODR位为0000，进入掉电模式）。
 */
bool st1vafe3bx_stop(void) {
    uint8_t ctrl5_val = 0;
    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_CTRL5, &ctrl5_val)) {
        return false;
    }
    ctrl5_val &= (uint8_t)(~ST1VAFE3BX_CTRL5_ODR_Msk);
    ctrl5_val |= ST1VAFE3BX_ODR_POWER_DOWN;
    return st1vafe3bx_write_register(ST1VAFE3BX_REG_CTRL5, ctrl5_val);
}

/* ==================== 默认参数配置 ==================== */

/**
 * @brief 配置加速度计默认参数（量程2g，带宽ODR/4）
 * @return 配置成功返回true，失败返回false
 */
static bool st1vafe3bx_config_accel_default(void) {
    ST1VAFE3BX_ACCEL_CONFIG default_accel_config = {
        .full_scale = ST1VAFE3BX_FS_2G,         // 量程：2g
        .bandwidth = ST1VAFE3BX_BW_quarter,        // 带宽：ODR/4         
    };
    return st1vafe3bx_config_accel(&default_accel_config);
}

/**
 * @brief 配置vAFE默认参数（差分模式x16增益、1GΩ阻抗、不启用高通滤波）
 * @return 配置成功返回true，失败返回false 
 */
static bool st1vafe3bx_config_vafe_default(void) {
    ST1VAFE3BX_VAFE_CONFIG default_vafe_config = {
        .mode = ST1VAFE3BX_VAFE_MODE_DIFFERENTIAL,      
        .gain = ST1VAFE3BX_VAFE_GAIN_16X,                 // 增益
        .input_impedance = ST1VAFE3BX_VAFE_ZIN_1GOHM,   // 1GΩ阻抗
        .vafe_only_mode = false,                          // 是否开启仅vafe模式
    };
    return st1vafe3bx_config_vafe(&default_vafe_config);
}
//强制复位
static bool st1vafe3bx_config_vafe_init(void) {
    ST1VAFE3BX_VAFE_CONFIG default_vafe_config = {
        .mode = ST1VAFE3BX_VAFE_MODE_FORCED_RESET ,      
        .gain = ST1VAFE3BX_VAFE_GAIN_16X,                // 增益
        .input_impedance = ST1VAFE3BX_VAFE_ZIN_1GOHM,   // 1GΩ阻抗
        .vafe_only_mode = false,                          // vAFE+加速度计同步模式
    };
    return st1vafe3bx_config_vafe(&default_vafe_config);
}

/**
 * @brief 配置FIFO默认参数（FIFO模式、水位64、混合数据、标准深度）
 * @return 配置成功返回true，失败返回false
 */
static bool st1vafe3bx_config_fifo_default(void) {
    ST1VAFE3BX_FIFO_CONFIG default_fifo_config = {
        .mode = ST1VAFE3BX_FIFO_MODE_CONTINUOUS,// 连续模式（满后覆盖旧数据，避免停采）
        .watermark_threshold = 64,              // 水位阈值：64个FIFO字，包含时间戳字和数据字
        .accel_only_fifo = false,               // 混合所有通道数据进入FIFO
        .double_depth_mode = false,             // 标准深度模式
        .timestamp_bdr = ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_1,
        .accel_bdr = ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_DEFAULT
    };
    return st1vafe3bx_config_fifo(&default_fifo_config);
}

/**
 * @brief 配置默认中断输出特性与使能INT引脚
 * @return 配置成功返回true，失败返回false
 */
static bool st1vafe3bx_config_interrupt_default(void) {
    ST1VAFE3BX_INTERRUPT_CONFIG default_interrupt_config = {
        .int_pin_enable = true,
        .timestamp_enable = true,
        .basic_int_pin_enable = false,
        .int_fifo_threshold_enable = true,
        .level_interrupt_lir = false,
        .interrupt_polarity_active_low = false,
        .od_output = false
    };
    return st1vafe3bx_config_interrupt(&default_interrupt_config);
}

/**
 * @brief 应用芯片默认配置，包含FIFO、中断、vAFE的完整初始化。
 * @return 配置成功返回true，失败返回false
 */
bool st1vafe3bx_apply_default_config(void) {

    // 配置加速度计默认参数
    if (!st1vafe3bx_config_accel_default()) {
        return false;
    }

    // 配置vAFE生物电势通道
    if (!st1vafe3bx_config_vafe_init()) {
        return false;
    }
    st1vafe3bx_delay_ms(1000);

    if (!st1vafe3bx_config_vafe_default()) {
        return false;
    }

    // 配置FIFO模式与水位阈值
    if (!st1vafe3bx_config_fifo_default()) {
        return false;
    }

    // 配置中断输出特性与使能INT引脚
    if (!st1vafe3bx_config_interrupt_default()) {
        return false;
    }

    return true;
}

/* ==================== 参数配置接口 ==================== */

/**
 * @brief 配置加速度计参数（支持FS、BW、BDU灵活配置）
 */
bool st1vafe3bx_config_accel(_IN_ ST1VAFE3BX_ACCEL_CONFIG *config) {
    if (config == NULL) {
        return false;
    }

    // CTRL5 寄存器：配置FS和带宽
    uint8_t ctrl5_val = 0;
    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_CTRL5, &ctrl5_val)) {
        return false;
    }
    // 配置量程（FS）
    ctrl5_val &= ~ST1VAFE3BX_CTRL5_FS_Msk;
    ctrl5_val |= (uint8_t)(config->full_scale & ST1VAFE3BX_CTRL5_FS_Msk);
    // 配置带宽（BW）
    ctrl5_val &= ~ST1VAFE3BX_CTRL5_BW_Msk;
    ctrl5_val |= (uint8_t)(config->bandwidth & ST1VAFE3BX_CTRL5_BW_Msk);

    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_CTRL5, ctrl5_val)) {
        return false;
    }


    return true;
}

/**
 * @brief 配置vAFE生物电势通道（模式、增益、阻抗）
 */
bool st1vafe3bx_config_vafe(_IN_ ST1VAFE3BX_VAFE_CONFIG *config) {
    if (config == NULL) {
        return false;
    }
    
    uint8_t vafe_cfg = 0x00;
    if (config->vafe_only_mode) {
        vafe_cfg |= ST1VAFE3BX_AH_BIO_CFG2_AH_BIO_EN_Msk;  //only_vafe模式
    } else {
        vafe_cfg &= (uint8_t)(~ST1VAFE3BX_AH_BIO_CFG2_AH_BIO_EN_Msk);
    }
    vafe_cfg |= (uint8_t)(config->mode & ST1VAFE3BX_AH_BIO_CFG2_MODE_Msk);
    vafe_cfg |= (uint8_t)(config->gain & ST1VAFE3BX_AH_BIO_CFG2_GAIN_Msk);
    vafe_cfg |= (uint8_t)(config->input_impedance & ST1VAFE3BX_AH_BIO_CFG2_ZIN_Msk);

    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_AH_BIO_CFG2, vafe_cfg)) {
        return false;
    }
    
    st1vafe3bx_delay_ms(10);
    return true;
}

/**
 * @brief 配置FIFO参数（模式、水位阈值、数据类型、深度模式）
 */
bool st1vafe3bx_config_fifo(_IN_ ST1VAFE3BX_FIFO_CONFIG *config) {
    if (config == NULL) {
        return false;
    }

    // CTRL4 寄存器：显式使能 FIFO_EN
    uint8_t ctrl4_val = 0;
    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_CTRL4, &ctrl4_val)) {
        return false;
    }
    ctrl4_val |= ST1VAFE3BX_CTRL4_FIFO_EN_Msk;
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_CTRL4, ctrl4_val)) {
        return false;
    }

    // 配置FIFO_CTRL:FIFO模式与深度模式
    uint8_t fifo_ctrl = 0;
    fifo_ctrl |= (uint8_t)(config->mode & ST1VAFE3BX_FIFO_CTRL_FIFO_MODE_Msk);
    if (config->double_depth_mode) {
        fifo_ctrl |= ST1VAFE3BX_FIFO_CTRL_FIFO_DEPTH_Msk;
    }
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_FIFO_CTRL, fifo_ctrl)) {
        return false;
    }

    // 配置FIFO_WTM:WTM与是否仅存accel数据
    if (config->watermark_threshold > ST1VAFE3BX_FIFO_WATERMARK_MAX) {
        return false;  // 阈值超出范围
    }
    uint8_t fifo_wtm = (uint8_t)(config->watermark_threshold & ST1VAFE3BX_FIFO_WTM_FTH_Msk);
    if (config->accel_only_fifo) {
        fifo_wtm |= ST1VAFE3BX_FIFO_WTM_XL_ONLY_FIFO_Msk;
    }
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_FIFO_WTM, fifo_wtm)) {
        return false;
    }

    // 配置FIFO_BATCH_DEC:时间戳与加速度计数据采样率
    uint8_t fifo_batch_dec = 0;
    fifo_batch_dec |= (uint8_t)(config->timestamp_bdr & ST1VAFE3BX_REG_FIFO_BATCH_DEC_DEC_TS_BATCH_Msk);
    fifo_batch_dec |= (uint8_t)(config->accel_bdr & ST1VAFE3BX_REG_FIFO_BATCH_DEC_BDR_XL_Msk);
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_FIFO_BATCH_DEC, fifo_batch_dec)) {
        return false;
    }

    st1vafe3bx_delay_ms(1);
    return true;
}

/**
 * @brief 配置中断输出特性与使能INT引脚
 * @param config 中断配置参数结构体指针
 * @return 配置成功返回true，失败返回false
 */
bool st1vafe3bx_config_interrupt(_IN_ ST1VAFE3BX_INTERRUPT_CONFIG *config) {
    if (config == NULL) {
        return false;
    }

    // CTRL1 寄存器 (10h)：INT_PIN_EN使能控制
    uint8_t ctrl1_val = 0;
    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_CTRL1, &ctrl1_val)) {
        return false;
    }
    if (config->int_pin_enable) {
        ctrl1_val |= ST1VAFE3BX_CTRL1_INT_PIN_EN_Msk;
    } else {
        ctrl1_val &= (uint8_t)(~ST1VAFE3BX_CTRL1_INT_PIN_EN_Msk);
    }
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_CTRL1, ctrl1_val)) {
        return false;
    }

    // PIN_CTRL 寄存器 (0Ch)：配置中断输出特性
    // H_LACTIVE [Bit 3]：中断极性控制，默认为0高电平有效
    // PP_OD [Bit 1]：输出类型控制，默认0推挽输出
    uint8_t pin_ctrl = 0x00;
    if (config->interrupt_polarity_active_low) {
        pin_ctrl |= ST1VAFE3BX_PIN_CTRL_H_LACTIVE_Msk;
    }
    if (config->od_output) {
        pin_ctrl |= ST1VAFE3BX_PIN_CTRL_PP_OD_Msk;
    }
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_PIN_CTRL, pin_ctrl)) {
        return false;
    }

    // INTERRUPT_CFG 寄存器 (17h)：配置中断模式与时间戳使能
    // LIR [Bit 1]：0=电平模式(条件消失后自动拉低), 1=锁存模式(读源寄存器清除)
    //INTERRUPTS_ENABLE是否开启basic_int
    uint8_t interrupt_cfg = 0x00;
    if (config->timestamp_enable) {
        interrupt_cfg |= ST1VAFE3BX_INTERRUPT_CFG_TIMESTAMP_EN_Msk;
    }
    if (config->basic_int_pin_enable) {
        interrupt_cfg |= ST1VAFE3BX_INTERRUPT_CFG_INTERRUPTS_ENABLE_Msk;
    }
    if (config->level_interrupt_lir) {
        interrupt_cfg |= ST1VAFE3BX_INTERRUPT_CFG_LIR_Msk;
    }
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_INTERRUPT_CFG, interrupt_cfg)) {
        return false;
    }

    // CTRL2 寄存器 (11h)：中断触发源配置
    // INT_FIFO_TH [Bit 5]：FIFO阈值中断源使能
    uint8_t ctrl2_val = 0;
    if (!st1vafe3bx_read_register(ST1VAFE3BX_REG_CTRL2, &ctrl2_val)) {
        return false;
    }
    if (config->int_fifo_threshold_enable) {
        ctrl2_val |= ST1VAFE3BX_CTRL2_INT_FIFO_TH_Msk;
    } else {
        ctrl2_val &= (uint8_t)(~ST1VAFE3BX_CTRL2_INT_FIFO_TH_Msk);
    }
    if (!st1vafe3bx_write_register(ST1VAFE3BX_REG_CTRL2, ctrl2_val)) {
        return false;
    }

    st1vafe3bx_interrupt_active_high = !(config->interrupt_polarity_active_low);
    if (config->int_pin_enable && !st1vafe3bx_init_interrupt_input()) {
        return false;
    }

    return true;
}

/* ==================== 中断输入与事件标志 ==================== */

/**
 * @brief 初始化CH585侧PA1中断输入并使能GPIOA中断
 */
bool st1vafe3bx_init_interrupt_input(void) {
    gpio_mode_config(ST1VAFE3BX_CH585_INT_GROUP, ST1VAFE3BX_CH585_INT_PIN, GPIO_MODE_IN_PULL_UP);

    R16_PA_INT_IF = (uint16_t)ST1VAFE3BX_CH585_INT_PIN;//RW1Z，写1清除中断标志
    gpio_interrupt_mode_config(
        ST1VAFE3BX_CH585_INT_GROUP,
        ST1VAFE3BX_CH585_INT_PIN,
        st1vafe3bx_interrupt_active_high ? GPIO_INTERRUPT_MODE_RISE_EDGE : GPIO_INTERRUPT_MODE_FALL_EDGE);

    pfic_enable_irq(IRQ_GPIO_A);
    return true;
}

/**
 * @brief 查询ST1中断事件是否到达
 */
__HIGH_CODE
bool st1vafe3bx_is_interrupt_pending(void) {
    return st1vafe3bx_interrupt_pending;
}

/**
 * @brief 查询ST1VAFE3BX中断脚当前电平（FIFO水位条件是否成立）。
 * @note INTERRUPT_CFG的LIR=0为电平模式：未读样本数>=水位FTH时引脚保持高电平，
 *       读空到水位以下后自动拉低，因此可用引脚电平兜底被漏掉的边沿中断。
 */
__HIGH_CODE
bool st1vafe3bx_is_fifo_watermark_asserted(void) {
    bool level = false;

    if (!st1vafe3bx_spi0_drdy_pin(&level)) {
        return false;
    }
    return level;
}

void st1vafe3bx_clear_interrupt_pending(void) {
    st1vafe3bx_interrupt_pending = false;
    R16_PA_INT_IF = (uint16_t)ST1VAFE3BX_CH585_INT_PIN;//RW1Z，写1清除中断标志
}

/**
 * @brief GPIOA中断服务函数：响应PA1并置位ST1中断事件
 */
__INTERRUPT
__HIGH_CODE
void GPIOA_IRQHandler(void) {
    if ((R16_PA_INT_IF & (uint16_t)ST1VAFE3BX_CH585_INT_PIN) != 0) {
        R16_PA_INT_IF = (uint16_t)ST1VAFE3BX_CH585_INT_PIN;//RW1Z，写1清除中断标志
        st1vafe3bx_interrupt_pending = true;//中断时间待处理标志，主机读取
    }
}

/* ==================== FIFO读取与解码 ==================== */

/**
 * @brief FIFO中断处理总函数
 */
bool st1vafe3bx_handle_interrupt_fifo(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *raw_samples,
    _OUT_ ST1VAFE3BX_DECODED_SAMPLE *decoded_samples) {
    if (raw_samples == NULL || decoded_samples == NULL) {
        return false;
    }

    if (!st1vafe3bx_interrupt_pending) {
        return false;
    }

    uint16_t total_read = 0;
    while (true) {
        uint8_t fifo_count = 0;
        if (!st1vafe3bx_get_fifo_count(&fifo_count)) {
            return false;
        }
        if (fifo_count == 0) {
            break;
        }

        for (uint8_t i = 0; i < fifo_count; i++) {
            if (total_read >= 256U) {
                return false;
            }
            if (!st1vafe3bx_read_fifo_sample(&raw_samples[total_read])) {
                return false;
            }
            if (!st1vafe3bx_decode_fifo_sample(&raw_samples[total_read], &decoded_samples[total_read])) {
                return false;
            }
            total_read++;
        }
    }

    st1vafe3bx_interrupt_pending = false;
    return true;
}

/**
 * @brief 获取FIFO状态快照
 */
bool st1vafe3bx_get_fifo_status(_OUT_ ST1VAFE3BX_FIFO_STATUS *status) {
    uint8_t raw_status[2] = {0U};

    if (status == NULL) {
        return false;
    }

    /* STATUS1/2地址连续；一次事务读取可减少SPI片选和两寄存器的读取间隔。 */
    if (!st1vafe3bx_read_registers(
            ST1VAFE3BX_REG_FIFO_STATUS1,
            raw_status,
            (uint8_t)sizeof(raw_status))) {
        return false;
    }

    status->watermark_asserted =
        (raw_status[0] & ST1VAFE3BX_FIFO_STATUS1_WTM_MASK) != 0U;
    status->overrun =
        (raw_status[0] & ST1VAFE3BX_FIFO_STATUS1_OVR_MASK) != 0U;
    status->unread_words =
        (uint8_t)(raw_status[1] & ST1VAFE3BX_FIFO_STATUS2_FSS_MASK);
    return true;
}

/**
 * @brief 获取FIFO中的未读样本数
 */
bool st1vafe3bx_get_fifo_count(_OUT_ uint8_t *sample_count) {
    ST1VAFE3BX_FIFO_STATUS status = {0};

    if (sample_count == NULL || !st1vafe3bx_get_fifo_status(&status)) {
        return false;
    }

    *sample_count = status.unread_words;
    return true;
}

/**
 * @brief 从FIFO读取一个完整原始样本（Tag + 6字节原始数据，不解码）
 */
bool st1vafe3bx_read_fifo_sample(ST1VAFE3BX_RAW_SAMPLE *sample) {
    if (sample == NULL) {
        return false;
    }

    if (!st1vafe3bx_read_registers(
            ST1VAFE3BX_REG_FIFO_DATA_OUT_TAG,
            (uint8_t *)&sample->tag,
            ST1VAFE3BX_FIFO_SAMPLE_LENGTH)) {
        return false;
    }

    return true;
} 

/**
 * @brief 从FIFO按当前样本数批量读取原始样本（不解码）
 */
__HIGH_CODE
bool st1vafe3bx_read_fifo_burst(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *samples) {

    uint16_t total_read = 0;
    while (true) {
        uint8_t fifo_count = 0;
        if (!st1vafe3bx_get_fifo_count(&fifo_count)) {
            return false;
        }

        if (fifo_count == 0) {
            break;
        }

        for (uint8_t i = 0; i < fifo_count; i++) {
            if (total_read >= 256U) {
                return false;
            }
            if (!st1vafe3bx_read_fifo_sample(&samples[total_read])) {
                return false;
            }
            total_read++;
        }
    }

    return true;
}

/**
 * @brief 对单个原始FIFO样本进行解码
 */
__HIGH_CODE
bool st1vafe3bx_decode_fifo_sample(
    const ST1VAFE3BX_RAW_SAMPLE *raw_sample,
    ST1VAFE3BX_DECODED_SAMPLE *decoded_sample) {
    if (raw_sample == NULL || decoded_sample == NULL) {
        return false;
    }

    uint8_t tag_id_h5 = (uint8_t)(raw_sample->tag >> 3);

    /* TAG=0x04: 时间戳帧，仅缓存；下一条ACCEL_VAFE帧再组成完整解码样本 */
    if (tag_id_h5 == ST1VAFE3BX_FIFO_TAG_TIMESTAMP) {
        st1vafe3bx_pending_timestamp = ((uint32_t)raw_sample->data[5] << 24)
                                     | ((uint32_t)raw_sample->data[4] << 16)
                                     | ((uint32_t)raw_sample->data[3] << 8)
                                     | (uint32_t)raw_sample->data[2];
        st1vafe3bx_pending_timestamp_valid = true;
        return false;
    } else if (tag_id_h5 == ST1VAFE3BX_FIFO_TAG_ACCEL_VAFE) {
        uint16_t acc_x_12 = (uint16_t)(((uint16_t)(raw_sample->data[1] & 0x0FU) << 8) | raw_sample->data[0]);
        uint16_t acc_y_12 = (uint16_t)(((uint16_t)raw_sample->data[2] << 4) | ((raw_sample->data[1] >> 4) & 0x0FU));
        uint16_t acc_z_12 = (uint16_t)(((uint16_t)(raw_sample->data[4] & 0x0FU) << 8) | raw_sample->data[3]);
        uint16_t vafe_12 = (uint16_t)(((uint16_t)raw_sample->data[5] << 4) | ((raw_sample->data[4] >> 4) & 0x0FU));

        decoded_sample->x = st1vafe3bx_align_12bit_left(acc_x_12);
        decoded_sample->y = st1vafe3bx_align_12bit_left(acc_y_12);
        decoded_sample->z = st1vafe3bx_align_12bit_left(acc_z_12);
        decoded_sample->vafe_data = st1vafe3bx_align_12bit_left(vafe_12);
        if (st1vafe3bx_pending_timestamp_valid) {
            decoded_sample->timestamp = st1vafe3bx_pending_timestamp;
            st1vafe3bx_pending_timestamp_valid = false;
        } else {
            decoded_sample->timestamp = 0U;
        }
    } else {
        return false;
    }

    return true;
}

/**
 * @brief 对批量原始FIFO样本进行解码
 */
bool st1vafe3bx_decode_fifo_burst(
    _IN_ const ST1VAFE3BX_RAW_SAMPLE *raw_samples,
    _OUT_ ST1VAFE3BX_DECODED_SAMPLE *decoded_samples,
    uint16_t sample_count) {
    if (raw_samples == NULL || decoded_samples == NULL) {
        return false;
    }

    for (uint16_t i = 0; i < sample_count; i++) {
        if (!st1vafe3bx_decode_fifo_sample(&raw_samples[i], &decoded_samples[i])) {
            return false;
        }
    }

    return true;
}

/**
 * @brief FIFO中断处理总函数（DMA读取版）
 */
bool st1vafe3bx_handle_interrupt_fifo_dma(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *raw_samples,
    _OUT_ ST1VAFE3BX_DECODED_SAMPLE *decoded_samples) {
    if (raw_samples == NULL || decoded_samples == NULL) {
        return false;
    }

    if (!st1vafe3bx_interrupt_pending) {
        return false;
    }

    uint16_t sample_count = 0U;
    if (!st1vafe3bx_read_fifo_burst_dma(raw_samples, 256U, &sample_count)) {
        return false;
    }

    if (!st1vafe3bx_decode_fifo_burst(raw_samples, decoded_samples, sample_count)) {
        return false;
    }

    st1vafe3bx_interrupt_pending = false;
    return true;
}

bool st1vafe3bx_read_fifo_sample_dma(ST1VAFE3BX_RAW_SAMPLE *sample) {
    if (sample == NULL) {
        return false;
    }

    uint8_t raw[ST1VAFE3BX_FIFO_SAMPLE_LENGTH] __attribute__((aligned(4))) = {0};
    if (!st1vafe3bx_read_registers_dma(
            ST1VAFE3BX_REG_FIFO_DATA_OUT_TAG,
            raw,
            ST1VAFE3BX_FIFO_SAMPLE_LENGTH)) {
        return false;
    }

    sample->tag = raw[0];
    for (uint8_t i = 0; i < 6; i++) {
        sample->data[i] = raw[i + 1U];
    }

    return true;
}

__HIGH_CODE
static bool st1vafe3bx_read_fifo_burst_dma_internal(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *samples,
    _IN_ uint16_t max_samples,
    _OUT_ uint16_t *sample_count,
    _OUT_ ST1VAFE3BX_FIFO_READ_REPORT *report) {
    uint16_t capacity;
    uint16_t total_read = 0;

    if (report != NULL) {
        report->status_observed = false;
        report->max_unread_words = 0U;
        report->overrun_observed = false;
    }

    if (samples == NULL || sample_count == NULL || max_samples == 0U) {
        return false;
    }
    /* _IN_ 参数为 const，用局部变量做上限裁剪 */
    capacity = (max_samples > 256U) ? 256U : max_samples;

    *sample_count = 0U;
    while (true) {
        ST1VAFE3BX_FIFO_STATUS status = {0};
        uint8_t fifo_count;

        if (!st1vafe3bx_get_fifo_status(&status)) {
            return false;
        }
        fifo_count = status.unread_words;
        if (report != NULL) {
            report->status_observed = true;
            if (status.unread_words > report->max_unread_words) {
                report->max_unread_words = status.unread_words;
            }
            report->overrun_observed =
                report->overrun_observed || status.overrun;
        }

        if (fifo_count == 0U) {
            break;
        }

        if ((uint16_t)(total_read + fifo_count) > capacity) {
            /* 只读到调用方给的容量上限，剩余样本留在FIFO，下次再读 */
            fifo_count = (uint8_t)(capacity - total_read);
            if (fifo_count == 0U) {
                break;
            }
        }

        {
            uint16_t burst_bytes = (uint16_t)fifo_count * ST1VAFE3BX_FIFO_SAMPLE_LENGTH;
            uint8_t *dst = (uint8_t *)&samples[total_read];

            if (!st1vafe3bx_read_registers_dma(
                    ST1VAFE3BX_REG_FIFO_DATA_OUT_TAG,
                    dst,
                    burst_bytes)) {
                return false;
            }

            total_read = (uint16_t)(total_read + fifo_count);
        }
    }

    *sample_count = total_read;
    return true;
}

__HIGH_CODE
bool st1vafe3bx_read_fifo_burst_dma(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *samples,
    _IN_ uint16_t max_samples,
    _OUT_ uint16_t *sample_count) {
    return st1vafe3bx_read_fifo_burst_dma_internal(
        samples,
        max_samples,
        sample_count,
        NULL);
}

__HIGH_CODE
bool st1vafe3bx_read_fifo_burst_dma_with_report(
    _OUT_ ST1VAFE3BX_RAW_SAMPLE *samples,
    _IN_ uint16_t max_samples,
    _OUT_ uint16_t *sample_count,
    _OUT_ ST1VAFE3BX_FIFO_READ_REPORT *report) {
    if (report == NULL) {
        return false;
    }
    return st1vafe3bx_read_fifo_burst_dma_internal(
        samples,
        max_samples,
        sample_count,
        report);
}

/* ==================== 直接输出寄存器读取 ==================== */

/**
 * @brief 读取加速度计XYZ三轴原始数据
 */
bool st1vafe3bx_read_accel_data(_OUT_ int16_t *x, _OUT_ int16_t *y, _OUT_ int16_t *z) {
    if (x == NULL || y == NULL || z == NULL) {
        return false;
    }

    uint8_t data[6] = {0};
    if (!st1vafe3bx_read_registers(ST1VAFE3BX_REG_OUT_X_L, data, 6)) {
        return false;
    }

    if (!st1vafe3bx_bytes_to_int16(data[0], data[1], x)) {
        return false;
    }
    if (!st1vafe3bx_bytes_to_int16(data[2], data[3], y)) {
        return false;
    }
    if (!st1vafe3bx_bytes_to_int16(data[4], data[5], z)) {
        return false;
    }

    return true;
}

/**
 * @brief 读取vAFE生物电势原始数据（12位）
 */
bool st1vafe3bx_read_vafe_data(_OUT_ int16_t *vafe_data) {
    if (vafe_data == NULL) {
        return false;
    }

    uint8_t data[2] = {0};
    if (!st1vafe3bx_read_registers(ST1VAFE3BX_REG_OUT_AH_BIO_L, data, 2)) {
        return false;
    }

    return st1vafe3bx_bytes_to_int16(data[0], data[1], vafe_data);
}
