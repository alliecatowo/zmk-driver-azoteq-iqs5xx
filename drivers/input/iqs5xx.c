/*
 * Copyright (c) 2025 Mariano Uvalle
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs5xx

#include <stdlib.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "iqs5xx.h"

LOG_MODULE_REGISTER(iqs5xx, CONFIG_INPUT_LOG_LEVEL);

/*
 * Number of I2C retry attempts and inter-attempt backoff.
 * Mirrors the Linux kernel iqs5xx driver's pattern (drivers/input/touchscreen/iqs5xx.c
 * IQS5XX_NUM_RETRIES = 10). Reason from the Linux comment: the IQS5xx chip's
 * comm-window protocol means the first I2C transaction outside an open window
 * can fail; the chip then clock-stretches/recovers and accepts the next attempt.
 * Without retry, isolated NACKs cause silent config failures (e.g. flip-x not
 * landing) on fast power cycles where the comm window timing is unfavorable.
 */
#define IQS5XX_I2C_RETRIES 10
#define IQS5XX_I2C_RETRY_BACKOFF_US 250

static int iqs5xx_i2c_write_with_retry(const struct device *dev,
                                       const uint8_t *buf, size_t len) {
    const struct iqs5xx_config *config = dev->config;
    int ret;
    for (int i = 0; i < IQS5XX_I2C_RETRIES; i++) {
        ret = i2c_write_dt(&config->i2c, buf, len);
        if (ret == 0) {
            return 0;
        }
        k_usleep(IQS5XX_I2C_RETRY_BACKOFF_US);
    }
    return ret;
}

static int iqs5xx_i2c_write_read_with_retry(const struct device *dev,
                                            const uint8_t *tx_buf, size_t tx_len,
                                            uint8_t *rx_buf, size_t rx_len) {
    const struct iqs5xx_config *config = dev->config;
    int ret;
    for (int i = 0; i < IQS5XX_I2C_RETRIES; i++) {
        ret = i2c_write_read_dt(&config->i2c, tx_buf, tx_len, rx_buf, rx_len);
        if (ret == 0) {
            return 0;
        }
        k_usleep(IQS5XX_I2C_RETRY_BACKOFF_US);
    }
    return ret;
}

/*
 * Reads do NOT use retry. Runtime work-handler reads are expected to
 * occasionally NACK at end-of-comm-window — the driver handles this by
 * jumping to end_comm and waiting for the next RDY interrupt. Retrying
 * here would burn ~2.5ms per failed read, saturating the work queue
 * during chip transient states (e.g. post-fast-power-cycle ATI) and
 * causing the cursor to stop responding entirely. Single-shot reads
 * preserve the driver's intended fail-fast behavior.
 */
static int iqs5xx_read_reg16(const struct device *dev, uint16_t reg, uint16_t *val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[2];
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;

    ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, sizeof(buf));
    if (ret < 0) {
        return ret;
    }

    *val = (buf[0] << 8) | buf[1];
    return 0;
}

static int iqs5xx_read_reg8(const struct device *dev, uint16_t reg, uint8_t *val) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};

    return i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), val, 1);
}

/*
 * Diagnostic-only read helpers that DO retry. Use only at init for
 * NVD-state observability, NOT in the runtime hot loop. The retry is
 * needed because the very first read after ACK_RESET often NACKs while
 * the chip's comm window is still closing — without retry, the first
 * NVD read silently fails and we miss critical diagnostic data.
 */
static int iqs5xx_read_reg8_with_retry(const struct device *dev, uint16_t reg, uint8_t *val) {
    int ret;
    for (int i = 0; i < IQS5XX_I2C_RETRIES; i++) {
        ret = iqs5xx_read_reg8(dev, reg, val);
        if (ret == 0) return 0;
        k_usleep(IQS5XX_I2C_RETRY_BACKOFF_US);
    }
    return ret;
}

static int iqs5xx_read_reg16_with_retry(const struct device *dev, uint16_t reg, uint16_t *val) {
    int ret;
    for (int i = 0; i < IQS5XX_I2C_RETRIES; i++) {
        ret = iqs5xx_read_reg16(dev, reg, val);
        if (ret == 0) return 0;
        k_usleep(IQS5XX_I2C_RETRY_BACKOFF_US);
    }
    return ret;
}

/*
 * Burst read N bytes starting at `reg`. The IQS5xx auto-increments the
 * register address within a comm window, so a single i2c_write_read_dt
 * pulls a contiguous block. Used in the work handler to read all per-event
 * status + relative-coordinate registers in one transaction instead of 6
 * separate reads — cuts I2C-on-the-wire time substantially and, more
 * importantly, eliminates per-transaction START/RESTART overhead. Less
 * time spent in the work handler = fewer coalesced/missed RDY events =
 * smoother tracking. Single-shot, no retry — same fail-fast philosophy
 * as the per-register reads.
 */
static int iqs5xx_read_burst(const struct device *dev, uint16_t reg, uint8_t *buf, size_t len) {
    const struct iqs5xx_config *config = dev->config;
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};

    return i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, len);
}

/*
 * Writes DO use retry. Setup_device writes happen once at init and the
 * comm-window-timing-NACK they may hit benefits from a few retries to
 * survive a transient closed window. The wasted time on failure is
 * absorbed in the one-time init path, not the runtime hot loop.
 */
static int iqs5xx_write_reg16(const struct device *dev, uint16_t reg, uint16_t val) {
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};

    return iqs5xx_i2c_write_with_retry(dev, buf, sizeof(buf));
}

static int iqs5xx_write_reg8(const struct device *dev, uint16_t reg, uint8_t val) {
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

    return iqs5xx_i2c_write_with_retry(dev, buf, sizeof(buf));
}

static int iqs5xx_end_comm_window(const struct device *dev) {
    /* End-comm window deliberately NACKs by chip-protocol design — don't
     * retry, just send once. Driver callers ignore the return value. */
    const struct iqs5xx_config *config = dev->config;
    uint8_t buf[3] = {IQS5XX_END_COMM_WINDOW >> 8, IQS5XX_END_COMM_WINDOW & 0xFF, 0x00};

    return i2c_write_dt(&config->i2c, buf, sizeof(buf));
}

static void iqs5xx_button_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, button_release_work);

    // TODO: This loop should only deactivate one button.
    // Log a warning when that is not the case.
    for (int i = 0; i < 3; i++) {
        LOG_INF("Releasing synthetic button");
        if (data->buttons_pressed & BIT(i)) {
            input_report_key(data->dev, INPUT_BTN_0 + i, 0, true, K_FOREVER);
            // Turn off the bit.
            // NOTE: This is a potential race.
            data->buttons_pressed &= ~BIT(i);
        }
    }
}

static void iqs5xx_work_handler(struct k_work *work) {
    struct iqs5xx_data *data = CONTAINER_OF(work, struct iqs5xx_data, work);
    const struct device *dev = data->dev;
    const struct iqs5xx_config *config = dev->config;
    uint8_t sys_info_0, sys_info_1, gesture_events_0, gesture_events_1;
    int16_t rel_x, rel_y;
    int ret;

    /*
     * Single burst read of all per-event status + relative-coordinate
     * registers (0x000D..0x0015, 9 bytes). Chip auto-increments the
     * register pointer within a comm window. Order on the wire:
     *
     *   buf[0] 0x000D GESTURE_EVENTS_0
     *   buf[1] 0x000E GESTURE_EVENTS_1
     *   buf[2] 0x000F SYSTEM_INFO_0
     *   buf[3] 0x0010 SYSTEM_INFO_1
     *   buf[4] 0x0011 NUM_FINGERS
     *   buf[5] 0x0012 REL_X high byte
     *   buf[6] 0x0013 REL_X low byte
     *   buf[7] 0x0014 REL_Y high byte
     *   buf[8] 0x0015 REL_Y low byte
     *
     * Replaces 6 separate i2c_write_read_dt calls. Saves ~5 START/RESTART
     * overheads per RDY cycle and shortens time-in-work-handler — the
     * direct cause of coalesced/missed RDY events that show up as jittery
     * tracking and "straight line" gaps where multiple samples were lost.
     *
     * SHOW_RESET is checked AFTER the burst so we still get a clean ack
     * path on chip-reset signaling — buf contents are nonsense in that
     * state but we just discard them and ack.
     */
    uint8_t buf[9];
    ret = iqs5xx_read_burst(dev, IQS5XX_GESTURE_EVENTS_0, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed burst read of event registers: %d", ret);
        goto end_comm;
    }
    gesture_events_0 = buf[0];
    gesture_events_1 = buf[1];
    sys_info_0 = buf[2];
    sys_info_1 = buf[3];
    /* buf[4] = NUM_FINGERS — read but not currently consumed. */
    rel_x = (int16_t)((buf[5] << 8) | buf[6]);
    rel_y = (int16_t)((buf[7] << 8) | buf[8]);

    if (sys_info_0 & IQS5XX_SHOW_RESET) {
        LOG_INF("Device reset detected, sending ACK_RESET");
        iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);
        goto end_comm;
    }

    /*
     * RR_MISSED is the chip's own "I had a sample ready but the host
     * didn't read in time" indicator. Log when it fires so we can see if
     * tracking jank correlates with chip-side drops vs MCU-side stalls.
     * Should be rare in steady state; non-rare = report rate too high
     * for the I2C+work-queue path or BLE backpressure.
     */
    if (sys_info_1 & IQS5XX_RR_MISSED) {
        LOG_WRN("Chip reports RR_MISSED — sample dropped");
    }

    bool tp_movement = (sys_info_1 & IQS5XX_TP_MOVEMENT) != 0;
    bool scroll = (gesture_events_1 & IQS5XX_SCROLL) != 0;
    if (!scroll) {
        // Clear accumulators if we're not actively scrolling.
        data->scroll_x_acc = 0;
        data->scroll_y_acc = 0;
    }

    uint16_t button_code;
    bool button_pressed = false;
    if (gesture_events_0 & IQS5XX_SINGLE_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_0;
    } else if (gesture_events_1 & IQS5XX_TWO_FINGER_TAP) {
        button_pressed = true;
        button_code = INPUT_BTN_1;
    }

    bool hold_became_active = (gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && !data->active_hold;
    bool hold_released = !(gesture_events_0 & IQS5XX_PRESS_AND_HOLD) && data->active_hold;

    /* rel_x/rel_y already pulled in the burst read above. */

    // Handle movement and gestures.
    //
    // Each one of these branches needs to send the last report it makes as
    // sync to ensure that the input subsystem processes things in order.
    if (hold_became_active) {
        LOG_INF("Hold became active");
        input_report_key(dev, LEFT_BUTTON_CODE, 1, true, K_FOREVER);
        data->active_hold = true;
    } else if (hold_released) {
        LOG_INF("Hold became inactive");
        input_report_key(dev, LEFT_BUTTON_CODE, 0, true, K_FOREVER);
        data->active_hold = false;
    } else if (button_pressed) {
        // Cancel any pending release.
        k_work_cancel_delayable(&data->button_release_work);

        // Press the button immediately.
        input_report_key(dev, button_code, 1, true, K_FOREVER);
        data->buttons_pressed |= BIT(button_code - INPUT_BTN_0);

        // Schedule release after 100ms.
        k_work_schedule(&data->button_release_work, K_MSEC(100));
    } else if (scroll) {
        // TODO: Expose this divisor.
        int16_t scroll_div = 32;

        // Only one scrolling direction is valid at a time.
        // End the communication right after reporting the movement.
        if (rel_x != 0) {
            // By default the x axis is already "natural".
            if (!config->natural_scroll_x) {
                rel_x *= -1;
            }
            data->scroll_x_acc += rel_x;
            if (abs(data->scroll_x_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_HWHEEL, data->scroll_x_acc / scroll_div, true,
                                K_FOREVER);
                data->scroll_x_acc %= scroll_div;
            }
            goto end_comm;
        }
        if (rel_y != 0) {
            if (config->natural_scroll_y) {
                rel_y *= -1;
            }
            data->scroll_y_acc += rel_y;
            if (abs(data->scroll_y_acc) >= scroll_div) {
                input_report_rel(dev, INPUT_REL_WHEEL, data->scroll_y_acc / scroll_div, true,
                                 K_FOREVER);
                data->scroll_y_acc %= scroll_div;
            }

            goto end_comm;
        }
    } else if (tp_movement) {
        if (rel_x != 0 || rel_y != 0) {
            input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
        }
    }

end_comm:
    // End communication window.
    iqs5xx_end_comm_window(dev);
}

static void iqs5xx_rdy_handler(const struct device *port, struct gpio_callback *cb,
                               gpio_port_pins_t pins) {
    struct iqs5xx_data *data = CONTAINER_OF(cb, struct iqs5xx_data, rdy_cb);

    k_work_submit(&data->work);
}

static int iqs5xx_setup_device(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    int ret;

    // ACK any pending reset signal first.
    //
    // After the NRST drive in iqs5xx_init (or any chip-side reset event),
    // the chip raises SHOW_RESET (bit 7 of SYS_INFO_0) and continues
    // firing RDY interrupts until the host acknowledges the reset by
    // writing ACK_RESET (bit 7 of SYSTEM_CONTROL_0 / register 0x0431).
    // Without this ack, the chip floods RDY events at high rate, the
    // work queue saturates, and BLE split forwarding starves — whole
    // keyboard appears to lock up. stelmakhdigital's TPS43 driver does
    // this handshake; AYM1607 had the ack write only inside the work
    // handler AFTER several register reads, so any read NACK skipped
    // it. We do it unconditionally first thing here. If chip wasn't
    // actually reset, the write is harmless (datasheet says ACK_RESET
    // bit is self-clearing).
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONTROL_0, IQS5XX_ACK_RESET);
    if (ret < 0) {
        LOG_ERR("Failed to ACK_RESET at setup: %d", ret);
        return ret;
    }
    k_msleep(10);

    /*
     * READ-ONLY NVD DIAGNOSTIC — captures chip's pre-write state into
     * data->diag for later logging by the delayed diagnostic work.
     * USB CDC isn't enumerated yet at this point, so direct LOG_INF
     * here would be lost; we defer logging to ~3s post-init.
     */
    k_msleep(50);
    {
        struct iqs5xx_data *data = dev->data;
        struct iqs5xx_diagnostic_state *d = &data->diag;
        iqs5xx_read_reg16_with_retry(dev, 0x0000, &d->pre_product_number);
        iqs5xx_read_reg16_with_retry(dev, 0x0002, &d->pre_project_number);
        iqs5xx_read_reg8_with_retry(dev, 0x0004, &d->pre_major_version);
        iqs5xx_read_reg8_with_retry(dev, 0x0005, &d->pre_minor_version);
        iqs5xx_read_reg16_with_retry(dev, 0x066E, &d->pre_x_resolution);
        iqs5xx_read_reg16_with_retry(dev, 0x0670, &d->pre_y_resolution);
        iqs5xx_read_reg8_with_retry(dev, 0x0632, &d->pre_filter_settings);
        iqs5xx_read_reg8_with_retry(dev, 0x0633, &d->pre_xy_static_beta);
        iqs5xx_read_reg8_with_retry(dev, 0x0637, &d->pre_bottom_beta);
        iqs5xx_read_reg8_with_retry(dev, 0x0638, &d->pre_lower_speed);
        iqs5xx_read_reg16_with_retry(dev, 0x0639, &d->pre_upper_speed);
        iqs5xx_read_reg8_with_retry(dev, 0x0586, &d->pre_idle_mode_timeout);
        iqs5xx_read_reg8_with_retry(dev, IQS5XX_SYSTEM_CONFIG_0, &d->pre_system_config_0);
        iqs5xx_read_reg8_with_retry(dev, IQS5XX_XY_CONFIG_0, &d->pre_xy_config_0);
    }

    // Clear SETUP_COMPLETE before any other config writes.
    //
    // The IQS5xx series treats register writes as advisory once SETUP_COMPLETE
    // is set: the chip continues using whatever configuration was active when
    // the bit was set, ignoring subsequent writes. On a fast power-cycle (e.g.
    // brief USB unplug/replug, MCU reset without full chip POR) the chip can
    // wake with SETUP_COMPLETE retained from the previous session. Without
    // clearing it first, any config writes done here — including XY_CONFIG_0
    // (flip-x/flip-y/switch-xy) — silently no-op on the chip. Result: cursor
    // axes can come up wrong despite firmware being correct, and only a long
    // (10s+) full power cycle restores expected behavior.
    //
    // Writing 0 to SYSTEM_CONFIG_0 first guarantees the chip is in
    // configuration mode regardless of prior state. ~10ms settle gives the
    // chip time to acknowledge the mode change before subsequent writes.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_0, 0);
    if (ret < 0) {
        LOG_ERR("Failed to clear SETUP_COMPLETE: %d", ret);
        return ret;
    }
    k_msleep(10);

    // Enable event mode and trackpad events.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_1,
                            IQS5XX_EVENT_MODE | IQS5XX_TP_EVENT | IQS5XX_GESTURE_EVENT);
    if (ret < 0) {
        LOG_ERR("Failed to configure event mode: %d", ret);
        return ret;
    }

    /*
     * Filter configuration registers (BOTTOM_BETA 0x0637, STATIONARY_THRESH
     * 0x0672, FILTER_SETTINGS 0x0632) are intentionally NOT written here.
     *
     * Per Azoteq IQS5xx-B000 datasheet §5.9, the dynamic IIR filter uses
     * register 0x0633 (XY_static_beta) as the MAX-filtering beta at slow
     * speeds — not 0x0637. Setting only BOTTOM_BETA without also writing
     * 0x0633, LOWER_SPEED (0x0638), and UPPER_SPEED (0x0639) puts the chip
     * into a half-configured dynamic-IIR state with undefined behavior at
     * low speeds (POR for 0x0633 is undocumented; likely 0 = no filter).
     * That is the root cause of the slow-drag jitter symptom we observed.
     *
     * Linux mainline iqs5xx.c, holykeebs QMK driver, and QMK upstream all
     * write zero filter registers — they trust the chip's NVD-baked tuning
     * from Azoteq's ConfigTool GUI shipped on the TPS43 module. We do the
     * same. If we ever need explicit filter control, write the full set
     * coherently per datasheet, not this partial config.
     *
     * See: INVESTIGATION-iqs5xx-feel.md in the parent zmk-corne repo for
     * the full research synthesis.
     */

    /* Active-mode report rate. Skipped when 0 to preserve chip default. */
    if (config->report_rate_active_ms > 0) {
        ret = iqs5xx_write_reg16(dev, IQS5XX_REPORT_RATE_ACTIVE, config->report_rate_active_ms);
        if (ret < 0) {
            LOG_ERR("Failed to set active report rate: %d", ret);
            return ret;
        }
    }
    if (config->report_rate_idle_touch_ms > 0) {
        ret = iqs5xx_write_reg16(dev, IQS5XX_REPORT_RATE_IDLE_TOUCH,
                                 config->report_rate_idle_touch_ms);
        if (ret < 0) {
            LOG_ERR("Failed to set idle-touch report rate: %d", ret);
            return ret;
        }
    }
    if (config->report_rate_idle_ms > 0) {
        ret = iqs5xx_write_reg16(dev, IQS5XX_REPORT_RATE_IDLE, config->report_rate_idle_ms);
        if (ret < 0) {
            LOG_ERR("Failed to set idle report rate: %d", ret);
            return ret;
        }
    }

    /* Write XY_STATIC_BETA when configured. This is the slow-speed
     * max-filter beta of the dynamic IIR. NVD on our TPS43 module ships
     * as 0 = filter freezes at slow speed = "moves in blocks" quantization.
     * Moderate value (~150-200) gives smoothing without freezing.
     * Setting to 0 leaves chip at NVD default. */
    if (config->xy_static_beta > 0) {
        ret = iqs5xx_write_reg8(dev, IQS5XX_XY_STATIC_BETA, config->xy_static_beta);
        if (ret < 0) {
            LOG_ERR("Failed to set XY static beta: %d", ret);
            return ret;
        }
    }

    /* Disable LP1/LP2 sleep transitions when configured. Datasheet §4.2:
     * 0xFF = never timeout. Prevents filter-state discontinuity that
     * surfaces as a "first-touch is sluggish" feel after the chip wakes. */
    if (config->disable_idle_timeout) {
        ret = iqs5xx_write_reg8(dev, IQS5XX_IDLE_MODE_TIMEOUT, 0xFF);
        if (ret < 0) {
            LOG_ERR("Failed to disable idle timeout: %d", ret);
            return ret;
        }
    }

    uint8_t single_finger_gestures = 0;
    single_finger_gestures |= config->one_finger_tap ? IQS5XX_SINGLE_TAP : 0;
    single_finger_gestures |= config->press_and_hold ? IQS5XX_PRESS_AND_HOLD : 0;
    // Configure single finger gestures.
    ret = iqs5xx_write_reg8(dev, IQS5XX_SINGLE_FINGER_GESTURES_CONF, single_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure single finger gestures: %d", ret);
        return ret;
    }

    // Configure the hold time for the press and hold gesture.
    ret = iqs5xx_write_reg16(dev, IQS5XX_HOLD_TIME, config->press_and_hold_time);
    if (ret < 0) {
        LOG_ERR("Failed to configure the hold time: %d", ret);
        return ret;
    }

    uint8_t two_finger_gestures = 0;
    two_finger_gestures |= config->two_finger_tap ? IQS5XX_TWO_FINGER_TAP : 0;
    two_finger_gestures |= config->scroll ? IQS5XX_SCROLL : 0;
    // Configure multi finger gestures.
    ret = iqs5xx_write_reg8(dev, IQS5XX_MULTI_FINGER_GESTURES_CONF, two_finger_gestures);
    if (ret < 0) {
        LOG_ERR("Failed to configure multi finger gestures: %d", ret);
        return ret;
    }

    // Configure axes.
    uint8_t xy_config = 0;
    xy_config |= config->flip_x ? IQS5XX_FLIP_X : 0;
    xy_config |= config->flip_y ? IQS5XX_FLIP_Y : 0;
    xy_config |= config->switch_xy ? IQS5XX_SWITCH_XY_AXIS : 0;
    xy_config |= config->palm_reject ? IQS5XX_PALM_REJECT : 0;
    ret = iqs5xx_write_reg8(dev, IQS5XX_XY_CONFIG_0, xy_config);
    if (ret < 0) {
        LOG_ERR("Failed to configure axes: %d", ret);
        return ret;
    }

    /* Final SYSTEM_CONFIG_0 write. SETUP_COMPLETE + WDT always; REATI +
     * ALP_REATI when configured (default true). Linux/QMK both enable
     * REATI — chip won't autonomously re-calibrate baseline drift without
     * these bits, which produces the "slow-drag drifts to silence" symptom
     * over long touches. */
    uint8_t system_config_0 = IQS5XX_SETUP_COMPLETE | IQS5XX_WDT;
    if (config->reati) {
        system_config_0 |= IQS5XX_REATI | IQS5XX_ALP_REATI;
    }
    ret = iqs5xx_write_reg8(dev, IQS5XX_SYSTEM_CONFIG_0, system_config_0);
    if (ret < 0) {
        LOG_ERR("Failed to configure system: %d", ret);
        return ret;
    }

    // End communication window.
    ret = iqs5xx_end_comm_window(dev);
    if (ret < 0) {
        LOG_ERR("Failed to end comm window during initialization: %d", ret);
        return ret;
    }

    /*
     * POST-WRITE readback — captures into data->diag for delayed log.
     */
    k_msleep(20);
    {
        struct iqs5xx_data *data = dev->data;
        struct iqs5xx_diagnostic_state *d = &data->diag;
        iqs5xx_read_reg16_with_retry(dev, 0x066E, &d->post_x_resolution);
        iqs5xx_read_reg16_with_retry(dev, 0x0670, &d->post_y_resolution);
        iqs5xx_read_reg8_with_retry(dev, 0x0633, &d->post_xy_static_beta);
        iqs5xx_read_reg8_with_retry(dev, 0x0586, &d->post_idle_mode_timeout);
        iqs5xx_read_reg8_with_retry(dev, IQS5XX_SYSTEM_CONFIG_0, &d->post_system_config_0);
        iqs5xx_read_reg8_with_retry(dev, IQS5XX_XY_CONFIG_0, &d->post_xy_config_0);
        iqs5xx_end_comm_window(dev);
        d->ready = true;
    }

    return 0;
}

/*
 * Delayed diagnostic logger. Fires ~3s after init, by which time USB CDC
 * is enumerated and any host-side log capture (cat) is connected. Prints
 * the snapshot taken during setup_device.
 */
static void iqs5xx_diagnostic_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct iqs5xx_data *data = CONTAINER_OF(dwork, struct iqs5xx_data, diagnostic_work);
    struct iqs5xx_diagnostic_state *d = &data->diag;

    if (!d->ready) {
        LOG_WRN("Diagnostic state not ready");
        return;
    }

    LOG_INF("=== IQS5xx PRE-WRITE NVD readback ===");
    LOG_INF("PRODUCT_NUMBER (0x0000) = %u", d->pre_product_number);
    LOG_INF("PROJECT_NUMBER (0x0002) = %u", d->pre_project_number);
    LOG_INF("VERSION (0x0004/0x0005) = %u.%u", d->pre_major_version, d->pre_minor_version);
    LOG_INF("X_RESOLUTION (0x066E) = %u", d->pre_x_resolution);
    LOG_INF("Y_RESOLUTION (0x0670) = %u", d->pre_y_resolution);
    LOG_INF("FILTER_SETTINGS (0x0632) = 0x%02x", d->pre_filter_settings);
    LOG_INF("XY_STATIC_BETA (0x0633) = %u", d->pre_xy_static_beta);
    LOG_INF("BOTTOM_BETA (0x0637) = %u", d->pre_bottom_beta);
    LOG_INF("LOWER_SPEED (0x0638) = %u", d->pre_lower_speed);
    LOG_INF("UPPER_SPEED (0x0639) = %u", d->pre_upper_speed);
    LOG_INF("IDLE_MODE_TIMEOUT (0x0586) = %u", d->pre_idle_mode_timeout);
    LOG_INF("SYSTEM_CONFIG_0 (0x058E) = 0x%02x", d->pre_system_config_0);
    LOG_INF("XY_CONFIG_0 (0x0669) = 0x%02x", d->pre_xy_config_0);
    LOG_INF("=== IQS5xx POST-WRITE readback ===");
    LOG_INF("X_RESOLUTION = %u (was %u)", d->post_x_resolution, d->pre_x_resolution);
    LOG_INF("Y_RESOLUTION = %u (was %u)", d->post_y_resolution, d->pre_y_resolution);
    LOG_INF("XY_STATIC_BETA = %u (was %u)", d->post_xy_static_beta, d->pre_xy_static_beta);
    LOG_INF("IDLE_MODE_TIMEOUT = %u (was %u)", d->post_idle_mode_timeout, d->pre_idle_mode_timeout);
    LOG_INF("SYSTEM_CONFIG_0 = 0x%02x (was 0x%02x)", d->post_system_config_0, d->pre_system_config_0);
    LOG_INF("XY_CONFIG_0 = 0x%02x (was 0x%02x)", d->post_xy_config_0, d->pre_xy_config_0);
    LOG_INF("=== END diagnostic ===");
}

static int iqs5xx_init(const struct device *dev) {
    const struct iqs5xx_config *config = dev->config;
    struct iqs5xx_data *data = dev->data;
    int ret;

    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    data->dev = dev;
    k_work_init(&data->work, iqs5xx_work_handler);
    k_work_init_delayable(&data->button_release_work, iqs5xx_button_release_work_handler);
    k_work_init_delayable(&data->diagnostic_work, iqs5xx_diagnostic_work_handler);

    // Configure reset GPIO if available.
    if (config->reset_gpio.port) {
        if (!gpio_is_ready_dt(&config->reset_gpio)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }

        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }

        // Reset the device.
        // Hold NRST asserted for 10ms (datasheet specifies >=150us; 10ms is
        // generous and matches stelmakhdigital's TPS43 driver).
        // Then release and wait 250ms for the chip's internal ATI calibration
        // to complete. The IQS572 datasheet specifies ATI takes ~150ms in
        // worst case; the previous 10ms wait was below this and caused
        // setup_device writes to fire during the chip's ATI window where
        // they could be NACKed silently. With the I2C retry helpers above,
        // shorter waits would also work, but a generous wait avoids
        // burning retry attempts during the deterministic ATI period.
        gpio_pin_set_dt(&config->reset_gpio, 1);
        k_msleep(10);
        gpio_pin_set_dt(&config->reset_gpio, 0);
        k_msleep(250);
    }

    // Configure RDY GPIO.
    if (!gpio_is_ready_dt(&config->rdy_gpio)) {
        LOG_ERR("RDY GPIO not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY GPIO: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->rdy_cb, iqs5xx_rdy_handler, BIT(config->rdy_gpio.pin));
    ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
    if (ret < 0) {
        LOG_ERR("Failed to add RDY callback: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure RDY interrupt: %d", ret);
        return ret;
    }

    // Wait for device to be ready.
    k_msleep(100);

    // Setup device configuration.
    ret = iqs5xx_setup_device(dev);
    if (ret < 0) {
        LOG_ERR("Failed to setup device: %d", ret);
        return ret;
    }

    data->initialized = true;
    LOG_INF("IQS5xx trackpad initialized");

    /* Schedule diagnostic dump 3s out — by then USB CDC has enumerated
     * and any host-side log capture is connected, so the snapshot we
     * took during setup_device actually reaches the host. */
    k_work_schedule(&data->diagnostic_work, K_SECONDS(3));

    return 0;
}

// Replace CONFIG_INPUT_INIT_PRIORITY with the azoteq specific value.
#define IQS5XX_INIT(n)                                                                             \
    static struct iqs5xx_data iqs5xx_data_##n;                                                     \
    static const struct iqs5xx_config iqs5xx_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                           \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                               \
        .one_finger_tap = DT_INST_PROP(n, one_finger_tap),                                         \
        .press_and_hold = DT_INST_PROP(n, press_and_hold),                                         \
        .two_finger_tap = DT_INST_PROP(n, two_finger_tap),                                         \
        .scroll = DT_INST_PROP(n, scroll),                                                         \
        .natural_scroll_x = DT_INST_PROP(n, natural_scroll_x),                                     \
        .natural_scroll_y = DT_INST_PROP(n, natural_scroll_y),                                     \
        .press_and_hold_time = DT_INST_PROP_OR(n, press_and_hold_time, 250),                       \
        .switch_xy = DT_INST_PROP(n, switch_xy),                                                   \
        .flip_x = DT_INST_PROP(n, flip_x),                                                         \
        .flip_y = DT_INST_PROP(n, flip_y),                                                         \
        .bottom_beta = DT_INST_PROP_OR(n, bottom_beta, 5),                                         \
        .stationary_threshold = DT_INST_PROP_OR(n, stationary_threshold, 5),                       \
        .report_rate_active_ms = DT_INST_PROP_OR(n, report_rate_active_ms, 10),                    \
        .report_rate_idle_touch_ms = DT_INST_PROP_OR(n, report_rate_idle_touch_ms, 10),            \
        .report_rate_idle_ms = DT_INST_PROP_OR(n, report_rate_idle_ms, 10),                        \
        .disable_idle_timeout = DT_INST_PROP_OR(n, disable_idle_timeout, true),                    \
        .palm_reject = DT_INST_PROP_OR(n, palm_reject, true),                                      \
        .reati = DT_INST_PROP_OR(n, reati, true),                                                  \
        .xy_static_beta = DT_INST_PROP_OR(n, xy_static_beta, 0),                                   \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, iqs5xx_init, NULL, &iqs5xx_data_##n, &iqs5xx_config_##n, POST_KERNEL, \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS5XX_INIT)
