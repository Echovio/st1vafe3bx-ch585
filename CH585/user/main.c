#include "board.h"
#include "hal.h"
#include "ble_peripheral.h"
#include "data_pipe_service.h"
#include "state_led.h"
#include "ch58x_clock.h"
#include "ch58x_adc.h"
#include "ch58x_gpio.h"
#include "ch58x_power.h"
#include "ch58x_rtc.h"
#include "ch58x_timer.h"
#include "ch58x_uart.h"
#include "core_riscv_pfic.h"
#include "st1vafe3bx.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define ST1_FIFO_BUFFER_CAPACITY 256U
#define BLE_IDLE_MAX_DURATION_10S 30U
#define BATTERY_SAMPLE_INTERVAL_MS 30000U
#define BATTERY_CONFIRM_INTERVAL_MS 2000U
#define BATTERY_LOW_MV 3300U
#define BATTERY_LOW_CONFIRM_COUNT 3U
#define BATTERY_ADC_SAMPLE_COUNT 10U

#define ST1_STREAM_FRAME_SIZE 12U
/* 每批最多40帧，12字节紧凑帧满批为480字节；保持装满即发与原有重试策略。
 * 标称800Hz时约20次通知/秒，实际多节点稳定性仍需实机验证。 */
#define ST1_TX_BATCH_MAX_FRAMES 40U
#define ST1_FRAME_HEADER_0 0xA5U
#define ST1_FRAME_HEADER_1 0x5BU
#define ST1_MLC_RESULT_FRAME_HEADER 0xC1U
#define ST1_MLC_RESULT_FRAME_SIZE 3U
#define ST1_CONTROL_FRAME_HEADER 0xD0U
#define ST1_CONTROL_CMD_SET_MLC_PROFILE 0x01U
#define ST1_CONTROL_CMD_SET_MODE 0x02U
#define ST1_CONTROL_FRAME_SIZE 3U
#define ST1_ACK_FRAME_HEADER 0xE0U
#define ST1_ACK_FRAME_SIZE 5U
#define ST1_ACK_STATUS_OK 0x00U
#define ST1_ACK_STATUS_ERROR 0x01U
#define ST1_ACK_DETAIL_NONE 0x00U
#define ST1_ACK_DETAIL_INVALID_MODE 0x01U
#define ST1_ACK_DETAIL_START_FAILED 0x02U
#define ST1_ACK_NOTIFY_RETRY_COUNT 8U
#define ST1_NOTIFY_DEFAULT_RETRY_COUNT 2U
/* 数据长时间不足一整包时的兜底冲刷（正常数据流由"装满即发"触发，不再提前冲刷半包） */
#define ST1_STREAM_BATCH_IDLE_FLUSH_MS 200U
/* 发送失败的批次保留待重试；重试间隔（毫秒）用于避免主循环空转 */
#define ST1_STREAM_BATCH_RETRY_MS 2U
/* main.c串口诊断打印开关：0=开启，1=关闭；可通过编译参数覆盖默认值。
 * 修改后重新编译并烧录。 */
#ifndef ST1_UART_PRINT_DISABLE
#define ST1_UART_PRINT_DISABLE 1
#endif
#if (ST1_UART_PRINT_DISABLE != 0) && (ST1_UART_PRINT_DISABLE != 1)
#error "ST1_UART_PRINT_DISABLE must be 0 or 1"
#endif
#define ST1_DIAGNOSTICS_PRINT_INTERVAL_MS 10000U
#define ST1_DIAGNOSTICS_UART_BAUD_RATE 115200U
#define ATT_NOTIFICATION_OVERHEAD 3U
#define ATT_LEGACY_PAYLOAD_BYTES 20U
#define ST1_APP_MODE_IDLE 0U
#define ST1_APP_MODE_FIFO_TRAINING 1U
#define ST1_APP_MODE_MLC_CLASSIFY  2U
#define BOARD_POWER_ENABLE_PA_PIN GPIO_PIN_6
#define BOARD_POWER_ENABLE_PB_PIN GPIO_PIN_0
/* 13.56MHz 时钟源下单段1s的计数值，双闪各段时长按此基准换算。 */
#define BOARD_POWER_PB_BLINK_TIMER_TICKS 13560000U
/* 双闪节拍：亮50ms → 灭100ms → 亮50ms → 灭1800ms，合计2s，占空比5%。
 * 采用短脉冲双闪而非长亮，在更低的平均电流下仍保留足够提示性。 */
#define BOARD_POWER_PB_BLINK_ON_TICKS (BOARD_POWER_PB_BLINK_TIMER_TICKS / 20U)
#define BOARD_POWER_PB_BLINK_GAP_TICKS (BOARD_POWER_PB_BLINK_TIMER_TICKS / 10U)
#define BOARD_POWER_PB_BLINK_OFF_TICKS (BOARD_POWER_PB_BLINK_TIMER_TICKS * 9U / 5U)
/* 双闪相位：0=首闪点亮中，1=闪间熄灭，2=次闪点亮中，3=长熄灭。 */
static volatile uint8_t s_pb_blink_phase = 0U;

/* 默认模式优先级：ST1_DEFAULT_MODE > 兼容旧宏ST1_APP_MODE > FIFO训练模式。 */
#ifndef ST1_DEFAULT_MODE
#ifdef ST1_APP_MODE
#define ST1_DEFAULT_MODE ST1_APP_MODE
#else
#define ST1_DEFAULT_MODE ST1_APP_MODE_FIFO_TRAINING
#endif
#endif

#if (ST1_DEFAULT_MODE != ST1_APP_MODE_FIFO_TRAINING) && \
        (ST1_DEFAULT_MODE != ST1_APP_MODE_MLC_CLASSIFY)
#error "ST1_DEFAULT_MODE must be ST1_APP_MODE_FIFO_TRAINING or ST1_APP_MODE_MLC_CLASSIFY"
#endif

_Static_assert((ST1_STREAM_FRAME_SIZE * ST1_TX_BATCH_MAX_FRAMES) <= 509U,
    "ST1 BLE stream batch exceeds max notification payload");

static volatile bool s_stream_notifications_enabled = false;
static volatile bool s_ble_connected = false;
/* 上电启动BLE/断开连接时记录，用于复核框架超时（RTC计数不是按2^32回绕）。 */
static uint32_t s_ble_idle_start_rtc = 0U;
static volatile bool s_mode_switch_pending = false;
static volatile uint8_t s_pending_mode = ST1_DEFAULT_MODE;
static volatile uint8_t s_current_mode = ST1_APP_MODE_IDLE;
static volatile bool s_ack_pending = false;
static volatile uint8_t s_pending_ack_command = 0U;
static volatile uint8_t s_pending_ack_status = 0U;
static volatile uint8_t s_pending_ack_mode = 0U;
static volatile uint8_t s_pending_ack_detail = 0U;

static uint8_t s_stream_batch_buffer[ST1_STREAM_FRAME_SIZE * ST1_TX_BATCH_MAX_FRAMES] = {0};
static volatile uint8_t s_stream_batch_count = 0U;
static volatile uint8_t s_mlc_tree_mask = 0U;
static volatile bool s_stream_batch_pending = false;
static uint32_t s_stream_batch_retry_clock = 0U;
static uint8_t s_mlc_last_results[ST1VAFE3BX_MLC_TREE_MAX] = {0};
static bool s_mlc_last_result_valid[ST1VAFE3BX_MLC_TREE_MAX] = {false};
static volatile bool s_mlc_initial_report_pending = false;
static volatile bool s_mlc_profile_switch_pending = false;
static volatile uint8_t s_pending_mlc_profile = (uint8_t)ST1VAFE3BX_MLC_PROFILE_DF;
static uint32_t s_stream_batch_first_clock = 0U;
static uint32_t s_stream_batch_pending_since_clock = 0U;
static bool s_fifo_overrun_active = false;

typedef struct _ST1_RUNTIME_DIAGNOSTICS {
    uint32_t fifo_read_failures;
    uint32_t fifo_overrun_episodes;
    uint32_t fifo_raw_words;
    uint32_t fifo_timestamp_words;
    uint32_t fifo_decoded_samples;
    uint32_t fifo_unknown_words;
    uint32_t batch_pending_episodes;
    uint32_t batch_retry_attempts;
    uint32_t batch_pending_max_ticks;
    uint32_t gatt_success;
    uint32_t gatt_alloc_failures;
    uint32_t gatt_msg_buffer_not_available;
    uint32_t gatt_memory_errors;
    uint32_t gatt_not_connected;
    uint32_t gatt_invalid_mtu;
    uint32_t gatt_timeouts;
    uint32_t gatt_invalid_parameters;
    uint32_t gatt_other_errors;
    /* bStatus_t为uint8_t；按原始失败码累计，保留DG原有分类口径。 */
    uint32_t gatt_error_counts[UINT8_MAX + 1U];
    uint8_t fifo_max_unread_words;
    uint8_t batch_max_frames;
} ST1_RUNTIME_DIAGNOSTICS;

static ST1_RUNTIME_DIAGNOSTICS s_diagnostics = {0};

/*
 * BLE/Data Pipe callbacks and the TMOS main loop share these small state flags.
 * Keep callback writes to volatile command state; sensor restart, FIFO draining
 * and notification sends are performed from the main loop.
 */

/**
 * @brief 使能外部硬件供电保持引脚。
 * @note PA6持续拉高；PB0为指示灯，启动时拉高，后续由TMR0中断按双闪节拍驱动。
 */
static void board_power_enable_startup(void) {
    gpio_set_pin_output(GPIO_GROUP_A, BOARD_POWER_ENABLE_PA_PIN, GPIO_OUTPUT_HIGH);
    gpio_set_pin_output(GPIO_GROUP_B, BOARD_POWER_ENABLE_PB_PIN, GPIO_OUTPUT_HIGH);
    gpio_mode_config(GPIO_GROUP_A, BOARD_POWER_ENABLE_PA_PIN, GPIO_MODE_OUT_PP_5mA);
    gpio_mode_config(GPIO_GROUP_B, BOARD_POWER_ENABLE_PB_PIN, GPIO_MODE_OUT_PP_5mA);
}

static void board_power_pb_blink_timer_startup(void) {
    power_peripheral_clock_config(POWER_CLOCK_TMR0, true);
    /* 首次周期装载首闪点亮时长，与相位0及启动时的PB0高电平一致。 */
    (void)timer_0_timer_init(
        TIMER_0_3_CLOCK_SOURCE_13_56_MHZ,
        BOARD_POWER_PB_BLINK_ON_TICKS);
    timer_0_timer_clear_interrupt_flag();
    timer_0_timer_enable_interrupt();
    pfic_enable_irq(IRQ_TMR0);
}

/* BLE空闲或电池低压时关机；关机后只能复位/重新上电恢复。 */
static void app_idle_shutdown(void) {
    pfic_disable_irq(IRQ_TMR0);
    timer_0_timer_disable_interrupt();
    timer_0_timer_disable();
    timer_0_timer_clear_interrupt_flag();
    pfic_clear_irq_pending(IRQ_TMR0);
    power_peripheral_clock_config(POWER_CLOCK_TMR0, false);

    gpio_set_pin_output(GPIO_GROUP_B, BOARD_POWER_ENABLE_PB_PIN, GPIO_OUTPUT_HIGH);
    clock_delay_ms(500U);
    gpio_set_pin_output(GPIO_GROUP_B, BOARD_POWER_ENABLE_PB_PIN, GPIO_OUTPUT_LOW);

    /* 不使用中间件的GPIO唤醒入口；ALL枚举未包含USB2，需一并关闭。 */
    pfic_disable_all_interrupts();
    power_wake_source_config(
        (POWER_WAKE_SOURCE)(POWER_WAKE_SOURCE_ALL | POWER_WAKE_SOURCE_USB2),
        WAKE_UP_DELAY_LONG, false);
    /* CH585的IRQ编号位于前两组，屏蔽并清除挂起请求，避免WFI立即退出。 */
    PFIC->IRER[0] = UINT32_MAX;
    PFIC->IRER[1] = UINT32_MAX;
    PFIC->IPRR[0] = UINT32_MAX;
    PFIC->IPRR[1] = UINT32_MAX;
    power_shutdown_all();
    for (;;) {
        /* 关机路径不返回采集流程。 */
    }
}

/**
 * @brief 重置待发送批次缓存计数。
 * @note 只清计数，不清内存内容；后续写入会覆盖旧数据。
 */
static void stream_batch_reset(void) {
    if (s_stream_batch_pending) {
        uint32_t pending_ticks =
            TMOS_GetSystemClock() - s_stream_batch_pending_since_clock;
        if (pending_ticks > s_diagnostics.batch_pending_max_ticks) {
            s_diagnostics.batch_pending_max_ticks = pending_ticks;
        }
    }
    s_stream_batch_count = 0U;
    s_stream_batch_first_clock = 0U;
    s_stream_batch_pending = false;
    s_stream_batch_pending_since_clock = 0U;
}

static void diagnostics_record_gatt_status(bStatus_t status) {
    /* 发送路径只计数，不打印；成功不计入失败码直方图。 */
    if (status != SUCCESS) {
        s_diagnostics.gatt_error_counts[status]++;
    }
    switch (status) {
        case SUCCESS:
            s_diagnostics.gatt_success++;
            break;
        case MSG_BUFFER_NOT_AVAIL:
            s_diagnostics.gatt_msg_buffer_not_available++;
            break;
        case bleMemAllocError:
            s_diagnostics.gatt_memory_errors++;
            break;
        case bleNotConnected:
            s_diagnostics.gatt_not_connected++;
            break;
        case bleInvalidMtuSize:
            s_diagnostics.gatt_invalid_mtu++;
            break;
        case bleTimeout:
            s_diagnostics.gatt_timeouts++;
            break;
        case INVALIDPARAMETER:
            s_diagnostics.gatt_invalid_parameters++;
            break;
        default:
            s_diagnostics.gatt_other_errors++;
            break;
    }
}

/**
 * @brief 根据当前协商 MTU 计算单次 Notification 目标帧数。
 * @return 目标帧数，范围 [1, ST1_TX_BATCH_MAX_FRAMES]。
 */
static uint8_t stream_calc_target_frames_per_notification(void) {
    uint16_t mtu = ble_peripheral_get_mtu();
    uint16_t payload_bytes = ATT_LEGACY_PAYLOAD_BYTES;
    uint16_t frames = 1U;

    if (mtu > ATT_NOTIFICATION_OVERHEAD) {
        payload_bytes = (uint16_t)(mtu - ATT_NOTIFICATION_OVERHEAD);
    }

    frames = (uint16_t)(payload_bytes / ST1_STREAM_FRAME_SIZE);
    if (frames < 1U) {
        frames = 1U;
    }
    if (frames > ST1_TX_BATCH_MAX_FRAMES) {
        frames = ST1_TX_BATCH_MAX_FRAMES;
    }

    return (uint8_t)frames;
}

/**
 * @brief 将一个解码样本打包为 12 字节流帧（A5 5B）。
 * @param sample 已解码样本。
 * @param destination 长度至少 ST1_STREAM_FRAME_SIZE 字节的目标缓冲。
 */
static void stream_pack_sample_to_frame(
    const ST1VAFE3BX_DECODED_SAMPLE *sample,
    uint8_t *destination) {
    /* 驱动输出为12位左对齐int16；先转无符号再去掉补零，保留负数原始位型。
     * 数据区沿用ACCEL_VAFE FIFO的6字节排列，不改变驱动及上位机物理量单位。 */
    uint16_t accel_x = (uint16_t)sample->x >> 4;
    uint16_t accel_y = (uint16_t)sample->y >> 4;
    uint16_t accel_z = (uint16_t)sample->z >> 4;
    uint16_t vafe = (uint16_t)sample->vafe_data >> 4;

    destination[0] = ST1_FRAME_HEADER_0;
    destination[1] = ST1_FRAME_HEADER_1;
    destination[2] = (uint8_t)(sample->timestamp >> 24);
    destination[3] = (uint8_t)(sample->timestamp >> 16);
    destination[4] = (uint8_t)(sample->timestamp >> 8);
    destination[5] = (uint8_t)sample->timestamp;
    destination[6] = (uint8_t)accel_x;
    destination[7] = (uint8_t)(((accel_y & 0x0FU) << 4) | (accel_x >> 8));
    destination[8] = (uint8_t)(accel_y >> 4);
    destination[9] = (uint8_t)accel_z;
    destination[10] = (uint8_t)(((vafe & 0x0FU) << 4) | (accel_z >> 8));
    destination[11] = (uint8_t)(vafe >> 4);
}

/**
 * @brief 通过 BLE Notification 发送一段连续字节。
 * @param data 待发送数据。
 * @param length 数据长度。
 * @param retry_count 缓冲申请和发送失败时的重试次数；0 会按 1 次处理。
 * @return 发送成功返回 true，否则返回 false。
 */
__HIGH_CODE
static bool app_notify_bytes(
    const uint8_t *data,
    uint16_t length,
    uint8_t retry_count) {
    uint16_t real_length = 0U;
    uint8_t *notification_buffer = NULL;
    uint8_t attempts = (retry_count == 0U) ? 1U : retry_count;

    if (!s_stream_notifications_enabled) {
        return false;
    }

    for (uint8_t attempt = 0U; attempt < attempts; attempt++) {
        notification_buffer = data_pipe_service_get_notification_buffer(length, &real_length);
        if (notification_buffer != NULL && real_length >= length) {
            break;
        }
        s_diagnostics.gatt_alloc_failures++;
        if (notification_buffer != NULL) {
            data_pipe_service_release_notification_buffer(notification_buffer, real_length);
            notification_buffer = NULL;
        }
        TMOS_SystemProcess();
        if (!s_stream_notifications_enabled) {
            return false;
        }
    }

    if (notification_buffer == NULL) {
        return false;
    }

    tmos_memcpy(notification_buffer, data, length);
    for (uint8_t attempt = 0U; attempt < attempts; attempt++) {
        bStatus_t status =
            data_pipe_service_notification_status(notification_buffer, length);
        diagnostics_record_gatt_status(status);
        if (status == SUCCESS) {
            return true;
        }
        TMOS_SystemProcess();
        if (!s_stream_notifications_enabled) {
            data_pipe_service_release_notification_buffer(notification_buffer, length);
            return false;
        }
    }

    data_pipe_service_release_notification_buffer(notification_buffer, length);
    return false;
}

/**
 * @brief 按条件冲刷批次并发送 Notification。
 * @param force_flush 为 true 时强制发送当前批次；否则按目标帧数达到后发送。
 * @return 批次已发出（或被丢弃/无需发送）返回 true；发送失败且批次保留待重试返回 false。
 * @note 发送失败不再丢弃整批：保留批次并在下一轮重试，避免把链路瞬时拥塞放大成数据丢失。
 */
__HIGH_CODE
static bool stream_flush_batch_if_needed(bool force_flush) {
    uint16_t batch_length;

    if (s_stream_batch_count == 0U) {
        return true;
    }

    if (!force_flush &&
            s_stream_batch_count < stream_calc_target_frames_per_notification()) {
        return true;
    }

    if (!s_stream_notifications_enabled) {
        stream_batch_reset();
        return true;
    }

    batch_length = (uint16_t)s_stream_batch_count * ST1_STREAM_FRAME_SIZE;
    if (!app_notify_bytes(
        s_stream_batch_buffer,
        batch_length,
        ST1_NOTIFY_DEFAULT_RETRY_COUNT)) {
        if (!s_stream_batch_pending) {
            s_stream_batch_pending_since_clock = TMOS_GetSystemClock();
            s_diagnostics.batch_pending_episodes++;
        }
        s_stream_batch_pending = true;
        s_stream_batch_retry_clock = TMOS_GetSystemClock();
        return false;
    }

    stream_batch_reset();
    return true;
}

/**
 * @brief 将样本入批次队列，并在满足条件时触发发送。
 * @param sample 已解码样本。
 */
__HIGH_CODE
static void stream_enqueue_sample(const ST1VAFE3BX_DECODED_SAMPLE *sample) {
    uint8_t *dst;

    if (!s_stream_notifications_enabled) {
        return;
    }

    if (s_stream_batch_count >= ST1_TX_BATCH_MAX_FRAMES) {
        if (!stream_flush_batch_if_needed(true)) {
            /* 批次满且未能发出：本次不入队，等重试成功后继续 */
            return;
        }
    }

    if (!s_stream_notifications_enabled) {
        return;
    }

    if (s_stream_batch_count == 0U) {
        s_stream_batch_first_clock = TMOS_GetSystemClock();
    }

    dst = &s_stream_batch_buffer[(uint16_t)s_stream_batch_count * ST1_STREAM_FRAME_SIZE];
    stream_pack_sample_to_frame(sample, dst);
    s_stream_batch_count++;
    if (s_stream_batch_count > s_diagnostics.batch_max_frames) {
        s_diagnostics.batch_max_frames = s_stream_batch_count;
    }
    (void)stream_flush_batch_if_needed(false);
}

/**
 * @brief 批次冲刷超时处理。
 * @note 正常数据流下由“装满即发”触发（每条链路每个连接事件的发包容额有限，
 *       实测提前冲刷会使平均帧/包从 32 降到 29，等于白丢约 9% 的容量）；
 *       本函数只做两件事：①发送失败后的 2ms 限速重试；②数据长时间不足一整包
 *       （ST1_STREAM_BATCH_IDLE_FLUSH_MS）时冲刷尾批，避免无限等待。
 */
static void stream_process_batch_flush_timeout(void) {
    if (s_stream_batch_count == 0U || !s_stream_notifications_enabled) {
        return;
    }

    if (s_stream_batch_pending) {
        if ((TMOS_GetSystemClock() - s_stream_batch_retry_clock) >=
                MS1_TO_SYSTEM_TIME(ST1_STREAM_BATCH_RETRY_MS)) {
            s_stream_batch_retry_clock = TMOS_GetSystemClock();
            s_diagnostics.batch_retry_attempts++;
            (void)stream_flush_batch_if_needed(true);
        }
        return;
    }

    if ((TMOS_GetSystemClock() - s_stream_batch_first_clock) <
            MS1_TO_SYSTEM_TIME(ST1_STREAM_BATCH_IDLE_FLUSH_MS)) {
        return;
    }

    (void)stream_flush_batch_if_needed(true);
}

/**
 * @brief 通过BLE通知发送指定 MLC 决策树分类结果。
 * @param tree_id 决策树编号，范围1~4，对应MLC1~MLC4。
 * @param result MLCx_SRC 原始结果；标签来自当前动作配置的结果元数据。
 * @return 发送成功返回true，否则返回false。
 * @note 帧格式：0xC1, tree_id, raw_result。
 */
__HIGH_CODE
static bool mlc_notify_result(uint8_t tree_id, uint8_t result) {
    uint8_t frame[ST1_MLC_RESULT_FRAME_SIZE] = {
        ST1_MLC_RESULT_FRAME_HEADER,
        tree_id,
        result
    };

    return app_notify_bytes(
        frame,
        ST1_MLC_RESULT_FRAME_SIZE,
        ST1_NOTIFY_DEFAULT_RETRY_COUNT);
}

/**
 * @brief 通过BLE通知发送控制命令ACK。
 * @note 帧格式：0xE0, cmd, status, mode, detail。
 */
__HIGH_CODE
static bool app_notify_ack(
    uint8_t command,
    uint8_t status,
    uint8_t mode,
    uint8_t detail) {
    uint8_t frame[ST1_ACK_FRAME_SIZE] = {
        ST1_ACK_FRAME_HEADER,
        command,
        status,
        mode,
        detail
    };

    return app_notify_bytes(frame, ST1_ACK_FRAME_SIZE, ST1_ACK_NOTIFY_RETRY_COUNT);
}

static void app_clear_pending_ack(void) {
    s_ack_pending = false;
}

static void app_store_pending_ack(
    uint8_t command,
    uint8_t status,
    uint8_t mode,
    uint8_t detail) {
    s_pending_ack_command = command;
    s_pending_ack_status = status;
    s_pending_ack_mode = mode;
    s_pending_ack_detail = detail;
    s_ack_pending = true;
}

static bool app_send_or_queue_ack(
    uint8_t command,
    uint8_t status,
    uint8_t mode,
    uint8_t detail) {
    if (app_notify_ack(command, status, mode, detail)) {
        app_clear_pending_ack();
        return true;
    }

    app_store_pending_ack(command, status, mode, detail);
    return false;
}

static void app_process_pending_ack(void) {
    uint8_t command;
    uint8_t status;
    uint8_t mode;
    uint8_t detail;

    if (!s_ack_pending || !s_stream_notifications_enabled) {
        return;
    }

    command = s_pending_ack_command;
    status = s_pending_ack_status;
    mode = s_pending_ack_mode;
    detail = s_pending_ack_detail;

    if (app_notify_ack(command, status, mode, detail)) {
        app_clear_pending_ack();
    }
}

static void app_stop_stream_notifications(void) {
    s_stream_notifications_enabled = false;
    s_mlc_initial_report_pending = false;
    app_clear_pending_ack();
    stream_batch_reset();
}

static void app_mark_ble_inactive(void) {
    s_ble_connected = false;
    app_stop_stream_notifications();
}

/* PA7 上的 10 MΩ / 10 MΩ 分压：采样期间暂用 ADC，完成后恢复原配置。 */
static uint32_t app_battery_voltage_mv(void) {
    uint8_t saved_channel = R8_ADC_CHANNEL;
    uint8_t saved_cfg = R8_ADC_CFG;
    uint8_t saved_convert = R8_ADC_CONVERT & (RB_ADC_SAMPLE_TIME | RB_ADC_PGA_GAIN2);
    uint32_t sum = 0U;

    gpio_mode_config(GPIO_GROUP_A, GPIO_PIN_7, GPIO_MODE_IN_FLOATING);
    adc_set_sample_clock(ADC_SAMPLE_CLOCK_32M_4_MHz);
    adc_set_pga_gain(ADC_PGA_0);
    adc_set_sample_time(ADC_SAMPLE_TIME_7_TADC);
    R8_ADC_CFG &= ~(RB_ADC_OFS_TEST | RB_ADC_DIFF_EN);
    adc_enable_input_buffer();
    adc_enable_power();
    clock_delay_us(10U);
    int16_t offset = adc_data_calibration_rough();
    adc_set_channel(ADC_SINGLE_CHANNEL_11);
    (void)adc_execute_single_conversion(false);
    for (uint8_t i = 0U; i < BATTERY_ADC_SAMPLE_COUNT; ++i) {
        int32_t sample = (int32_t)adc_execute_single_conversion(false) + offset;
        if (sample < 0) sample = 0;
        if (sample > 4095) sample = 4095;
        sum += (uint32_t)sample;
    }

    /* 默认关闭 ADC；若此前被其他功能使用，则恢复其配置。 */
    R8_ADC_CFG = saved_cfg;
    R8_ADC_CHANNEL = saved_channel;
    R8_ADC_CONVERT = saved_convert;
    uint32_t average = (sum + BATTERY_ADC_SAMPLE_COUNT / 2U) / BATTERY_ADC_SAMPLE_COUNT;
    return (average * 2100U + 1024U) / 2048U;
}

static void app_battery_monitor(void) {
    static bool first_sample = true;
    static uint8_t low_count = 0U;
    static uint32_t last_sample_clock = 0U;
    uint32_t now = TMOS_GetSystemClock();
    uint32_t interval_ms = low_count ? BATTERY_CONFIRM_INTERVAL_MS : BATTERY_SAMPLE_INTERVAL_MS;
    if (!first_sample && (now - last_sample_clock) < MS1_TO_SYSTEM_TIME(interval_ms)) {
        return;
    }
    first_sample = false;
    last_sample_clock = now;
    if (app_battery_voltage_mv() >= BATTERY_LOW_MV) {
        low_count = 0U;
        return;
    }
    if (++low_count >= BATTERY_LOW_CONFIRM_COUNT) {
        state_led_set_status(STATE_LED_STATUS_IDLE);
        app_mark_ble_inactive();
        app_idle_shutdown();
    }
}

static void mlc_result_cache_reset(void) {
    memset(s_mlc_last_results, 0, sizeof(s_mlc_last_results));
    memset(s_mlc_last_result_valid, 0, sizeof(s_mlc_last_result_valid));
}

/**
 * @brief 读取 ST1 FIFO，解码有效样本并推送到流发送队列。
 * @param raw_samples   FIFO 原始样本缓冲。
 * @param decoded_sample 解码样本临时对象。
 * @param max_samples   本次最多读取的样本数（不超过剩余可发送空间，避免读出来又丢掉）。
 */
__HIGH_CODE
static void sensor_drain_fifo_and_stream(
    ST1VAFE3BX_RAW_SAMPLE *raw_samples,
    ST1VAFE3BX_DECODED_SAMPLE *decoded_sample,
    uint16_t max_samples) {
    uint16_t sample_count = 0U;
    ST1VAFE3BX_FIFO_READ_REPORT report = {0};
    bool read_ok;

    read_ok = st1vafe3bx_read_fifo_burst_dma_with_report(
        raw_samples,
        max_samples,
        &sample_count,
        &report);
    if (report.status_observed) {
        if (report.max_unread_words > s_diagnostics.fifo_max_unread_words) {
            s_diagnostics.fifo_max_unread_words = report.max_unread_words;
        }
        if (report.overrun_observed) {
            if (!s_fifo_overrun_active) {
                s_diagnostics.fifo_overrun_episodes++;
                s_fifo_overrun_active = true;
            }
        } else {
            s_fifo_overrun_active = false;
        }
    }
    if (!read_ok) {
        s_diagnostics.fifo_read_failures++;
        return;
    }
    s_diagnostics.fifo_raw_words += sample_count;

    for (uint16_t i = 0; i < sample_count; i++) {
        uint8_t tag_id_h5 = (uint8_t)(raw_samples[i].tag >> 3);
        if (!st1vafe3bx_decode_fifo_sample(&raw_samples[i], decoded_sample)) {
            if (tag_id_h5 == ST1VAFE3BX_FIFO_TAG_TIMESTAMP) {
                /* 时间戳帧是中间态，继续读取下一帧。 */
                s_diagnostics.fifo_timestamp_words++;
                continue;
            }
            s_diagnostics.fifo_unknown_words++;
            break;
        }

        if (tag_id_h5 == ST1VAFE3BX_FIFO_TAG_ACCEL_VAFE) {
            s_diagnostics.fifo_decoded_samples++;
            stream_enqueue_sample(decoded_sample);
        }
    }

    st1vafe3bx_clear_interrupt_pending();
}

/**
 * @brief 按需上报指定 MLC 决策树结果。
 * @note force_report用于连接后的初始同步；非强制时仅结果变化才发送。
 */
__HIGH_CODE
static bool sensor_report_mlc_tree_result(
    uint8_t tree_id,
    uint8_t result,
    bool force_report) {
    uint8_t result_index = (uint8_t)(tree_id - 1U);

    if (!force_report &&
            s_mlc_last_result_valid[result_index] &&
            s_mlc_last_results[result_index] == result) {
        return false;
    }

    if (!mlc_notify_result(tree_id, result)) {
        return false;
    }

    s_mlc_last_results[result_index] = result;
    s_mlc_last_result_valid[result_index] = true;
    return true;
}

/**
 * @brief 读取MLC结果并上报，force_report为true时强制发送（用于初始同步），
 *        为false时仅在结果变化时发送（用于中断触发）。
 */
__HIGH_CODE
static void sensor_report_mlc_results(bool force_report) {
    uint8_t results[ST1VAFE3BX_MLC_TREE_MAX] = {0};

    if (!st1vafe3bx_read_valid_mlc_results(s_mlc_tree_mask, results)) {
        return;
    }

    for (uint8_t tree_id = 1U; tree_id <= ST1VAFE3BX_MLC_TREE_MAX; tree_id++) {
        uint8_t tree_bit = (uint8_t)(1U << (tree_id - 1U));
        if ((s_mlc_tree_mask & tree_bit) == 0U) {
            continue;
        }

        sensor_report_mlc_tree_result(
            tree_id,
            results[tree_id - 1U],
            force_report);
    }
}

static void mlc_request_initial_report_if_ready(void) {
    if (s_stream_notifications_enabled && s_mlc_tree_mask != 0U) {
        s_mlc_initial_report_pending = true;
    }
}

static bool app_mode_is_valid(uint8_t mode) {
    return (mode == ST1_APP_MODE_FIFO_TRAINING ||
            mode == ST1_APP_MODE_MLC_CLASSIFY);
}

static bool mlc_profile_from_command_id(
    uint8_t command_id,
    ST1VAFE3BX_MLC_PROFILE_ID *profile_id) {
    switch (command_id) {
        case 0U:
            *profile_id = ST1VAFE3BX_MLC_PROFILE_DF;
            return true;
        case 1U:
            *profile_id = ST1VAFE3BX_MLC_PROFILE_PF;
            return true;
        case 2U:
            *profile_id = ST1VAFE3BX_MLC_PROFILE_INV;
            return true;
        case 3U:
            *profile_id = ST1VAFE3BX_MLC_PROFILE_EV;
            return true;
        default:
            return false;
    }
}

static void app_handle_control_message(uint16_t length, const uint8_t *data) {
    ST1VAFE3BX_MLC_PROFILE_ID profile_id = ST1VAFE3BX_MLC_PROFILE_DF;

    if (length < ST1_CONTROL_FRAME_SIZE) {
        return;
    }

    if (data[0] != ST1_CONTROL_FRAME_HEADER) {
        return;
    }

    if (data[1] == ST1_CONTROL_CMD_SET_MODE) {
        if (!app_mode_is_valid(data[2])) {
            app_store_pending_ack(
                ST1_CONTROL_CMD_SET_MODE,
                ST1_ACK_STATUS_ERROR,
                s_current_mode,
                ST1_ACK_DETAIL_INVALID_MODE);
            return;
        }

        s_pending_mode = data[2];
        s_mode_switch_pending = true;
        return;
    }

    if (data[1] == ST1_CONTROL_CMD_SET_MLC_PROFILE) {
        if (s_current_mode != ST1_APP_MODE_MLC_CLASSIFY) {
            return;
        }
        if (!mlc_profile_from_command_id(data[2], &profile_id)) {
            return;
        }

        s_pending_mlc_profile = (uint8_t)profile_id;
        s_mlc_profile_switch_pending = true;
    }
}

static void mlc_process_pending_profile_switch(void) {
    ST1VAFE3BX_MLC_PROFILE_ID profile_id = ST1VAFE3BX_MLC_PROFILE_DF;

    if (!s_mlc_profile_switch_pending) {
        return;
    }

    profile_id = (ST1VAFE3BX_MLC_PROFILE_ID)s_pending_mlc_profile;
    s_mlc_profile_switch_pending = false;

    if (!st1vafe3bx_reload_mlc_profile(profile_id)) {
        s_mlc_tree_mask = 0U;
        s_mlc_initial_report_pending = false;
        mlc_result_cache_reset();
        return;
    }

    s_mlc_tree_mask = st1vafe3bx_get_mlc_tree_mask();
    mlc_result_cache_reset();
    mlc_request_initial_report_if_ready();
}

/**
 * @brief BLE 状态事件回调：维护连接状态与流发送状态。
 */
static void on_ble_state_event(BLE_PERIPHERAL_EVENT event) {
    switch (event) {
        case BLE_PERIPHERAL_EVENT_CONNECTED:
            state_led_set_status(STATE_LED_STATUS_STANDBY);
            s_ble_connected = true;
            break;
        case BLE_PERIPHERAL_EVENT_DISCONNECTED:
            state_led_set_status(STATE_LED_STATUS_STANDBY);
            app_mark_ble_inactive();
            s_ble_idle_start_rtc = rtc_get_32k_cycle_count();
            break;
        case BLE_PERIPHERAL_EVENT_ENTER_IDLE: {
            if (s_ble_connected) {
                break;
            }
            uint32_t now = rtc_get_32k_cycle_count();
            uint32_t elapsed = (now >= s_ble_idle_start_rtc)
                ? now - s_ble_idle_start_rtc
                : RTC_MAX_COUNT - s_ble_idle_start_rtc + now;
            if (elapsed < BLE_IDLE_MAX_DURATION_10S * 10U * SYSTEM_32K_CLOCK_FREQUENCY) {
                /* 框架秒数跨RTC回绕可能提前报空闲，未满5分钟则继续广播。 */
                ble_peripheral_advertisement_start();
                break;
            }
            state_led_set_status(STATE_LED_STATUS_IDLE);
            app_mark_ble_inactive();
            app_idle_shutdown();
            break;
        }
        case BLE_PERIPHERAL_EVENT_START_ADVERTISING:
            state_led_set_status(STATE_LED_STATUS_WORKING_LEVEL_1);
            break;
        case BLE_PERIPHERAL_EVENT_MTU_CHANGED:
        default:
            break;
    }
}

/**
 * @brief 数据管道服务事件回调：维护通知开关状态。
 */
static void on_data_pipe_event(
    DATA_PIPE_SERVICE_EVENT event,
    uint16_t *length,
    uint8_t *data) {
    switch (event) {
        case DATA_PIPE_SERVICE_EVENT_MESSAGE_READ:
            if (length != NULL) {
                *length = 0;
            }
            break;
        case DATA_PIPE_SERVICE_EVENT_STREAM_NOTIFICATION_START:
            s_stream_notifications_enabled = true;
            mlc_request_initial_report_if_ready();
            break;
        case DATA_PIPE_SERVICE_EVENT_STREAM_NOTIFICATION_STOP:
            app_stop_stream_notifications();
            break;
        case DATA_PIPE_SERVICE_EVENT_MESSAGE_WRITE:
            if (length != NULL && data != NULL) {
                app_handle_control_message(*length, data);
            }
            break;
        case DATA_PIPE_SERVICE_EVENT_STREAM_WRITE:
        default:
            break;
    }
}

/**
 * @brief 构建 BLE 外设初始化参数。
 * @return BLE_PERIPHERAL_PARAMETER 结构体。
 */
static BLE_PERIPHERAL_PARAMETER build_ble_parameter(void) {
    BLE_PERIPHERAL_PARAMETER ble_parameter = {
        .passkey = 0,
        .device_name = (uint8_t *)"Echo sEMG-IMU-N1",
        .device_name_length = (uint8_t)(sizeof("Echo sEMG-IMU-N1") - 1U),
        /* 连接参数请求值：间隔15~30ms(12~24 * 1.25ms)、无从机延迟、监督超时6s。
         * 7.5~12.5ms的请求Windows通常不采纳，30s监督超时也超出常规推荐范围。 */
        .connect_parameter = {
            .interval_min = 12,
            .interval_max = 24,
            .latency = 0,
            .supervision_timeout = 600,
        },
        .manufacturer_name_string = (uint8_t *)"Sipu Medical Group",
        .manufacturer_name_string_length = (uint16_t)(sizeof("Sipu Medical Group") - 1U),
        .hardware_revision_string = (uint8_t *)"v1.0.0",
        .hardware_revision_string_length = (uint16_t)(sizeof("v1.0.0") - 1U),
        .firmware_revision_string = (uint8_t *)"v1.0.0",
        .firmware_revision_string_length = (uint16_t)(sizeof("v1.0.0") - 1U),
        .software_revision_string = (uint8_t *)"v1.0.0",
        .software_revision_string_length = (uint16_t)(sizeof("v1.0.0") - 1U),
        .vender_id = 0xDCE5,
        .product_id = 0x0005,
        .manjor_version = 0x01,
        .minor_version = 0x00,
        .sub_minor_version = 0x00,
        .serial_number = 1,
        .idle_max_duration_10s = BLE_IDLE_MAX_DURATION_10S,
        .battery_voltage_monitor_channel = ADC_SINGLE_CHANNEL_NONE,
        .battery_voltage_monitor_sample_rate = 0,
    };

    return ble_parameter;
}

static void app_ble_start(void) {
    BLE_PERIPHERAL_PARAMETER ble_parameter = build_ble_parameter();

    ble_peripheral_init(ble_parameter);
    data_pipe_service_init(on_data_pipe_event);
    s_ble_idle_start_rtc = rtc_get_32k_cycle_count();
    /* 直接接收框架事件，避免LED中间件在回调返回后强制配置GPIO唤醒。 */
    ble_peripheral_start(on_ble_state_event);
}

static void app_reset_sensor_runtime_state(void) {
    stream_batch_reset();
    s_fifo_overrun_active = false;
    s_mlc_tree_mask = 0U;
    s_mlc_initial_report_pending = false;
    s_mlc_profile_switch_pending = false;
    mlc_result_cache_reset();
}

static bool app_start_sensor_mode(uint8_t mode, bool stop_first) {
    bool startup_ok;

    if (stop_first) {
        (void)st1vafe3bx_stop();
    }

    app_reset_sensor_runtime_state();
    if (mode == ST1_APP_MODE_FIFO_TRAINING) {
        startup_ok = st1vafe3bx_startup_fifo_mode();
    } else {
        startup_ok = st1vafe3bx_startup_mlc_mode();
        if (startup_ok) {
            s_mlc_tree_mask = st1vafe3bx_get_mlc_tree_mask();
            startup_ok = (s_mlc_tree_mask != 0U);
        }
    }

    if (!startup_ok) {
        (void)st1vafe3bx_stop();
        s_current_mode = ST1_APP_MODE_IDLE;
        return false;
    }

    s_current_mode = mode;
    st1vafe3bx_clear_interrupt_pending();
    if (mode == ST1_APP_MODE_MLC_CLASSIFY) {
        mlc_request_initial_report_if_ready();
    }
    return true;
}

static void app_process_pending_mode_switch(void) {
    uint8_t target_mode;
    bool startup_ok;

    if (!s_mode_switch_pending) {
        return;
    }

    target_mode = s_pending_mode;
    s_mode_switch_pending = false;

    if (target_mode == s_current_mode) {
        if (target_mode == ST1_APP_MODE_MLC_CLASSIFY) {
            mlc_request_initial_report_if_ready();
        }
        app_send_or_queue_ack(
            ST1_CONTROL_CMD_SET_MODE,
            ST1_ACK_STATUS_OK,
            s_current_mode,
            ST1_ACK_DETAIL_NONE);
        return;
    }

    startup_ok = app_start_sensor_mode(target_mode, true);
    app_send_or_queue_ack(
        ST1_CONTROL_CMD_SET_MODE,
        startup_ok ? ST1_ACK_STATUS_OK : ST1_ACK_STATUS_ERROR,
        startup_ok ? target_mode : s_current_mode,
        startup_ok ? ST1_ACK_DETAIL_NONE : ST1_ACK_DETAIL_START_FAILED);
}

/**
 * @brief FIFO训练模式单次轮询：处理 BLE 协议栈与传感器数据流。
 * @note ST1中断脚为电平模式，仅靠边沿标志会在漏掉一次边沿（如读取失败后
 *       直接返回）时永久停止读取，因此同时检查中断脚电平作为兜底。
 * @note 读取量按“批次剩余可发送空间”限量：发送失败时批次保留待重试，
 *       新样本留在传感器FIFO里；当前128字FIFO中每个样本另占一个时间戳字，
 *       按800Hz计算，空FIFO约可缓冲80ms；剩余可缓冲时间取决于当前水位。
 */
__HIGH_CODE
static void app_process_fifo_cycle(
    ST1VAFE3BX_RAW_SAMPLE *raw_samples,
    ST1VAFE3BX_DECODED_SAMPLE *decoded_sample) {
    uint8_t target_frames;

    if (!st1vafe3bx_is_interrupt_pending() &&
            !st1vafe3bx_is_fifo_watermark_asserted()) {
        return;
    }

    if (!s_stream_notifications_enabled) {
        /* 未订阅：照常读空FIFO丢弃，保持缓冲新鲜 */
        sensor_drain_fifo_and_stream(raw_samples, decoded_sample, ST1_FIFO_BUFFER_CAPACITY);
        return;
    }

    if (s_stream_batch_pending) {
        /* 批次待发：先不读FIFO，等重试成功后再继续搬运 */
        return;
    }

    target_frames = stream_calc_target_frames_per_notification();
    if (s_stream_batch_count >= target_frames) {
        return;
    }

    sensor_drain_fifo_and_stream(
        raw_samples,
        decoded_sample,
        (uint16_t)(target_frames - s_stream_batch_count));
}

/**
 * @brief MLC分类模式单次轮询：MLCx结果变化触发中断后读取分类结果。
 */
__HIGH_CODE
static void app_process_mlc_cycle(void) {
    mlc_process_pending_profile_switch();

    if (s_stream_notifications_enabled && s_mlc_initial_report_pending) {
        s_mlc_initial_report_pending = false;
        sensor_report_mlc_results(true);
    }

    if (!st1vafe3bx_is_interrupt_pending()) {
        return;
    }

    sensor_report_mlc_results(false);
    st1vafe3bx_clear_interrupt_pending();
}

static void app_process_current_mode(
    ST1VAFE3BX_RAW_SAMPLE *raw_samples,
    ST1VAFE3BX_DECODED_SAMPLE *decoded_sample) {
    switch (s_current_mode) {
        case ST1_APP_MODE_FIFO_TRAINING:
            app_process_fifo_cycle(raw_samples, decoded_sample);
            break;
        case ST1_APP_MODE_MLC_CLASSIFY:
            app_process_mlc_cycle();
            break;
        case ST1_APP_MODE_IDLE:
        default:
            break;
    }
}

#if !ST1_UART_PRINT_DISABLE
static uint32_t s_diagnostics_last_print_clock = 0U;

/**
 * @brief 诊断专用UART1输出，独立于DEBUG和printf的_write重定向。
 * @note 仅在主循环/启动阶段调用；保持原有同步输出方式，每行缓冲有界。
 */
static void app_diagnostics_printf(const char *format, ...)
    __attribute__((format(printf, 1, 2)));

static void app_diagnostics_printf(const char *format, ...) {
    char line[256];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length <= 0) {
        return;
    }
    if ((size_t)length >= sizeof(line)) {
        length = (int)sizeof(line) - 1;
        line[length - 1] = '\n';
    }
    uart_1_send_bytes((uint8_t *)line, (uint16_t)length);
}

static void app_diagnostics_init(void) {
    /* 开启诊断后固定使用PA9/TX、PA8/RX，不依赖dbg构建或DEBUG宏。 */
    uart_1_init(
        UART_PORT_1_PA_R8_T9,
        UART_FIFO_TRIGGER_NUMBER_4_BYTE,
        ST1_DIAGNOSTICS_UART_BAUD_RATE);
    app_diagnostics_printf("ST1 DIAG UART1 115200 ready\n");
}

/**
 * @brief 低频输出旁路诊断；由ST1_UART_PRINT_DISABLE控制，不写入Data Pipe。
 */
static void app_print_diagnostics_if_due(void) {
    uint32_t now = TMOS_GetSystemClock();
    uint32_t pending_max_ticks = s_diagnostics.batch_pending_max_ticks;
    BLE_PERIPHERAL_LINK_DIAGNOSTICS link = {0};

    if ((now - s_diagnostics_last_print_clock) <
            MS1_TO_SYSTEM_TIME(ST1_DIAGNOSTICS_PRINT_INTERVAL_MS)) {
        return;
    }
    s_diagnostics_last_print_clock = now;
    (void)ble_peripheral_get_link_diagnostics(&link);
    if (s_stream_batch_pending) {
        uint32_t current_pending_ticks =
            now - s_stream_batch_pending_since_clock;
        if (current_pending_ticks > pending_max_ticks) {
            pending_max_ticks = current_pending_ticks;
        }
    }

    app_diagnostics_printf("DQ fm=%u ov=%u rf=%u raw=%u dec=%u ts=%u tag=%u q=%u/%u pe=%u rr=%u pmax=%u\n",
        (unsigned int)s_diagnostics.fifo_max_unread_words,
        (unsigned int)s_diagnostics.fifo_overrun_episodes,
        (unsigned int)s_diagnostics.fifo_read_failures,
        (unsigned int)s_diagnostics.fifo_raw_words,
        (unsigned int)s_diagnostics.fifo_decoded_samples,
        (unsigned int)s_diagnostics.fifo_timestamp_words,
        (unsigned int)s_diagnostics.fifo_unknown_words,
        (unsigned int)s_stream_batch_count,
        (unsigned int)s_diagnostics.batch_max_frames,
        (unsigned int)s_diagnostics.batch_pending_episodes,
        (unsigned int)s_diagnostics.batch_retry_attempts,
        (unsigned int)pending_max_ticks);
    app_diagnostics_printf("DG ok=%u af=%u mb=%u me=%u nc=%u mtuE=%u to=%u ip=%u ot=%u\n",
        (unsigned int)s_diagnostics.gatt_success,
        (unsigned int)s_diagnostics.gatt_alloc_failures,
        (unsigned int)s_diagnostics.gatt_msg_buffer_not_available,
        (unsigned int)s_diagnostics.gatt_memory_errors,
        (unsigned int)s_diagnostics.gatt_not_connected,
        (unsigned int)s_diagnostics.gatt_invalid_mtu,
        (unsigned int)s_diagnostics.gatt_timeouts,
        (unsigned int)s_diagnostics.gatt_invalid_parameters,
        (unsigned int)s_diagnostics.gatt_other_errors);
    app_diagnostics_printf("DL c=%u h=%u mtu=%u phy=%u/%u ps=%u ci=%u lat=%u sup=%u cs=%u\n",
        (unsigned int)(link.connected ? 1U : 0U),
        (unsigned int)link.connection_handle,
        (unsigned int)link.mtu,
        (unsigned int)link.tx_phy,
        (unsigned int)link.rx_phy,
        (unsigned int)link.phy_update_status,
        (unsigned int)link.connection_interval,
        (unsigned int)link.connection_latency,
        (unsigned int)link.supervision_timeout,
        (unsigned int)link.connection_parameter_update_status);
    /* 每10秒仅输出曾出现的失败码；n为自启动累计次数，不在输出后清零。 */
    for (uint16_t status = 0U; status <= UINT8_MAX; status++) {
        uint32_t count = s_diagnostics.gatt_error_counts[status];
        if (count != 0U) {
            app_diagnostics_printf("DR code=0x%02X n=%u\n",
                (unsigned int)status,
                (unsigned int)count);
        }
    }
}

#else
/* 关闭时保留统一调用接口，空实现由编译器内联消除。 */
static inline void app_diagnostics_init(void) {}
static inline void app_print_diagnostics_if_due(void) {}
#endif /* !ST1_UART_PRINT_DISABLE */

/**
 * @brief 运行期主循环：根据当前模式处理FIFO流或MLC分类。
 * @note 按需求移除 WFI 休眠，持续运行主循环。
 */
__HIGH_CODE
void main_app_cycle(void) {
    ST1VAFE3BX_RAW_SAMPLE raw_samples[ST1_FIFO_BUFFER_CAPACITY] __ALIGNED(4) = {0};
    ST1VAFE3BX_DECODED_SAMPLE decoded_sample = {0};

    while (true) {
        TMOS_SystemProcess();
        app_battery_monitor();
        app_process_pending_mode_switch();
        app_process_pending_ack();
        stream_process_batch_flush_timeout();
        app_process_current_mode(raw_samples, &decoded_sample);
        app_print_diagnostics_if_due();
    }
}

/**
 * @brief TMR0按双闪节拍触发，驱动PB0指示灯完成"亮50ms→灭100ms→亮50ms→灭1800ms"。
 * @note 各段时长不同，因此在中断内重装 R32_TMR0_CNT_END 切换下一段周期（该寄存器
 *       是驱动中约定的周期设置入口，见 timer_0_capture_set_timeout）。
 *       首段与 board_power_enable_startup() 的初始高电平一致，上电即进入首闪。
 */
__INTERRUPT
__HIGH_CODE
void TMR0_IRQHandler(void) {
    if (timer_0_timer_get_interrupt_state()) {
        timer_0_timer_clear_interrupt_flag();
        switch (s_pb_blink_phase) {
            case 0U: {
                /* 首闪点亮结束，进入闪间熄灭。 */
                gpio_set_pin_output(GPIO_GROUP_B, BOARD_POWER_ENABLE_PB_PIN, GPIO_OUTPUT_LOW);
                R32_TMR0_CNT_END = BOARD_POWER_PB_BLINK_GAP_TICKS;
                s_pb_blink_phase = 1U;
                break;
            }
            case 1U: {
                /* 闪间熄灭结束，进入次闪点亮。 */
                gpio_set_pin_output(GPIO_GROUP_B, BOARD_POWER_ENABLE_PB_PIN, GPIO_OUTPUT_HIGH);
                R32_TMR0_CNT_END = BOARD_POWER_PB_BLINK_ON_TICKS;
                s_pb_blink_phase = 2U;
                break;
            }
            case 2U: {
                /* 次闪点亮结束，进入长熄灭。 */
                gpio_set_pin_output(GPIO_GROUP_B, BOARD_POWER_ENABLE_PB_PIN, GPIO_OUTPUT_LOW);
                R32_TMR0_CNT_END = BOARD_POWER_PB_BLINK_OFF_TICKS;
                s_pb_blink_phase = 3U;
                break;
            }
            default: {
                /* 长熄灭结束，回到首闪点亮。 */
                gpio_set_pin_output(GPIO_GROUP_B, BOARD_POWER_ENABLE_PB_PIN, GPIO_OUTPUT_HIGH);
                R32_TMR0_CNT_END = BOARD_POWER_PB_BLINK_ON_TICKS;
                s_pb_blink_phase = 0U;
                break;
            }
        }
    }
}

int main(void) {
    power_dcdc_config(true);
    board_power_enable_startup();
    init_clock_and_debug_uart();
    app_diagnostics_init();
    board_power_pb_blink_timer_startup();
    hal_init();

    app_ble_start();
    app_battery_monitor();
    (void)app_start_sensor_mode(ST1_DEFAULT_MODE, false);
    main_app_cycle();
}
