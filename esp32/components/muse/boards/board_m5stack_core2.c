/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * M5Stack Core2 (K010, v1.0): ESP32-D0WDQ6-V3 (classic ESP32, 16 MB flash,
 * 8 MB quad PSRAM), 2" 320x240 ILI9342C/E touch LCD (FT6336U), NS4168
 * speaker amp, PDM mic, AXP192 PMU and a CP2104 USB bridge. The BSP drives
 * the display, touch, speaker and PMU rails; the mic (which the BSP doesn't
 * cover) is driven here, and PWR (the PMU's PEK key, the only button) talks.
 * Pins and rails are from Espressif's BSP
 * (https://github.com/espressif/esp-bsp/tree/master/bsp/m5stack_core_2) and
 * M5Unified (src/M5Unified.inl, utility/power/AXP192_Class.inl); the panel
 * and touch setup from M5GFX (src/M5GFX.cpp); PEK/battery register numbers
 * from M5Unified's AXP192 driver and lewisxhe/AXP202X_Library.
 */
#include <math.h>

#include "bsp/esp-bsp.h"
#include "driver/i2c_master.h"
#include "driver/i2s_pdm.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "muse_audio.h"
#include "muse_board.h"
#include "muse_mem.h"

#include "axp192_pek.h"

#define AXP_ADDR 0x34
#define AXP_REG_ID 0x03
#define AXP_ID_192 0x03
#define AXP_REG_STATUS 0x00        /* bit5 VBUS present, bit2 charging */
#define AXP_REG_INTEN3 0x42        /* PEK short/long IRQ enables */
#define AXP_REG_INTSTS3 0x46       /* PEK short/long IRQ status */
#define AXP_REG_PEK_SET 0x36       /* PEK timings; bits 1:0 are the 4/6/8/10 s shutdown delay */
#define AXP_REG_BATT_V 0x78        /* 12-bit battery voltage, 1.1 mV/LSB */
#define AXP_REG_OFF_CTL 0x32

#define MIC_CLK GPIO_NUM_0         /* PDM mic clock, shared with the speaker WS */
#define MIC_DATA GPIO_NUM_34
#define MIC_GAIN_OFFSET_DB 6       /* as on the StickC Plus2 */

#define PMU_KEY_EVERY 2            /* poll the PMU over I2C every 20 ms */

static const char *TAG = "board";
static i2c_master_dev_handle_t s_axp;
static bool s_pmu;
static axp192_pek_t s_pek;
static esp_codec_dev_handle_t s_spk, s_mic;
static i2s_chan_handle_t s_mic_rx;
static bool s_mic_on;
static int s_mic_gain_q8 = 256;
static int32_t s_mic_dc[2];        /* each slot's zero level, x256 */
static bool s_mic_settled;         /* s_mic_dc set from a first read */

static esp_err_t axp_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_axp, &reg, 1, data, len, 100);
}

static esp_err_t axp_write(uint8_t reg, uint8_t val)
{
    const uint8_t buf[] = { reg, val };
    return i2c_master_transmit(s_axp, buf, sizeof(buf), 100);
}

static esp_err_t init(void)
{
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "i2c init");
    const i2c_device_config_t axp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bsp_i2c_get_handle(), &axp_cfg, &s_axp), TAG, "axp device");
    uint8_t id = 0;
    if (axp_read(AXP_REG_ID, &id, 1) != ESP_OK || id != AXP_ID_192) {
        ESP_LOGW(TAG, "AXP192 not found (id 0x%02x): no talk button or battery status", id);
        return ESP_OK;
    }
    /* Latch PEK short/long presses for poll_buttons(), clear stale state, and
     * stretch the hold-to-cut power from 4 s to 10 s. Leave the BSP's rails
     * alone. */
    uint8_t en = 0;
    ESP_RETURN_ON_ERROR(axp_read(AXP_REG_INTEN3, &en, 1), TAG, "pek irq read");
    ESP_RETURN_ON_ERROR(axp_write(AXP_REG_INTEN3, en | AXP192_PEK_SHORT | AXP192_PEK_LONG), TAG, "pek irq enable");
    ESP_RETURN_ON_ERROR(axp_write(AXP_REG_INTSTS3, AXP192_PEK_SHORT | AXP192_PEK_LONG), TAG, "pek irq clear");
    uint8_t pek = 0;
    ESP_RETURN_ON_ERROR(axp_read(AXP_REG_PEK_SET, &pek, 1), TAG, "pek timings read");
    ESP_RETURN_ON_ERROR(axp_write(AXP_REG_PEK_SET, (pek & ~0x03) | 0x03), TAG, "pek shutdown 10s");
    s_pmu = true;
    return ESP_OK;
}

static lv_display_t *display_start(lv_indev_t **touch)
{
    /* Two small internal DMA buffers, not the BSP's default full-height one.
     * Its LVGL port supplies byte swapping and the panel/touch orientation. */
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * 10,
        .double_buffer = true,
        .flags = { .buff_dma = true, .buff_spiram = false },
    };
    cfg.lvgl_port_cfg.task_affinity = MUSE_UI_CORE;
    cfg.lvgl_port_cfg.task_priority = MUSE_UI_PRIORITY;
    lv_display_t *disp = bsp_display_start_with_config(&cfg);
    if (!disp) {
        return NULL;
    }
    /* The BSP's backlight init rewrites the PEK timings; restore the 10 s
     * hold-to-cut power so a long talk hold never kills the board. */
    if (s_pmu) {
        uint8_t pek = 0;
        if (axp_read(AXP_REG_PEK_SET, &pek, 1) == ESP_OK) {
            axp_write(AXP_REG_PEK_SET, (pek & ~0x03) | 0x03);
        }
    }
    *touch = bsp_display_get_input_dev();
    return *touch ? disp : NULL;
}

static bool display_lock(int timeout_ms)
{
    /* Muse uses -1 for forever; esp_lvgl_port uses 0. */
    return bsp_display_lock(timeout_ms < 0 ? 0 : (uint32_t)timeout_ms);
}

static void set_brightness(int pct)
{
    bsp_display_brightness_set(pct);
}

static int mic_enable(const audio_codec_data_if_t *h, esp_codec_dev_type_t type, bool on)
{
    (void)h;
    (void)type;
    if (on != s_mic_on) {
        if ((on ? i2s_channel_enable(s_mic_rx) : i2s_channel_disable(s_mic_rx)) != ESP_OK) {
            return ESP_CODEC_DEV_DRV_ERR;
        }
        s_mic_on = on;
        s_mic_settled = false;
    }
    return ESP_CODEC_DEV_OK;
}

/*
 * Both PDM slots, as the 2-slot stream muse_audio reads; the mic answers on
 * one. The ESP32's PDM-to-PCM filter leaves a large DC offset, which the gain
 * would clip: each slot's zero level is tracked (a 10 Hz high-pass; M5Unified
 * tracks it too) and taken off first.
 */
static int mic_read(const audio_codec_data_if_t *h, uint8_t *data, int size)
{
    (void)h;
    size_t got;
    if (i2s_channel_read(s_mic_rx, data, size, &got, pdMS_TO_TICKS(1000)) != ESP_OK || got != (size_t)size) {
        return ESP_CODEC_DEV_READ_FAIL;
    }
    int16_t *s = (int16_t *)data;
    int n = size / 2;
    if (!s_mic_settled) {
        int32_t sum[2] = { 0, 0 };
        for (int i = 0; i < n; i++) {
            sum[i & 1] += s[i];
        }
        s_mic_dc[0] = sum[0] / (n / 2) * 256;
        s_mic_dc[1] = sum[1] / (n / 2) * 256;
        s_mic_settled = true;
    }
    for (int i = 0; i < n; i++) {
        int32_t *dc = &s_mic_dc[i & 1];
        *dc += (s[i] * 256 - *dc) >> 8;
        int v = (s[i] - (*dc >> 8)) * s_mic_gain_q8 >> 8;
        s[i] = v > INT16_MAX ? INT16_MAX : v < INT16_MIN ? INT16_MIN : v;
    }
    return ESP_CODEC_DEV_OK;
}

static void set_mic_gain(esp_codec_dev_handle_t mic, int db)
{
    (void)mic;
    s_mic_gain_q8 = (int)(256.0f * powf(10.0f, (db - MIC_GAIN_OFFSET_DB) / 20.0f));
}

static esp_err_t audio_init(esp_codec_dev_handle_t *spk, esp_codec_dev_handle_t *mic)
{
    /* Muse records two slots; the BSP's no-argument default is 22.05 kHz. */
    const i2s_std_config_t cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = BSP_I2S_MCLK,
            .bclk = BSP_I2S_SCLK,
            .ws = BSP_I2S_LCLK,
            .dout = BSP_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_RETURN_ON_ERROR(bsp_audio_init(&cfg), TAG, "speaker init");
    *spk = s_spk = bsp_audio_codec_speaker_init();
    if (!*spk) {
        return ESP_FAIL;
    }

    /* Only I2S0 does PDM. The mic's clock is 2.048 MHz, as M5Unified runs it. */
    i2s_chan_config_t mic_chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&mic_chan, NULL, &s_mic_rx), TAG, "mic channel");
    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(MUSE_AUDIO_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = { .clk = MIC_CLK, .din = MIC_DATA },
    };
    pdm_cfg.clk_cfg.dn_sample_mode = I2S_PDM_DSR_16S;
    ESP_RETURN_ON_ERROR(i2s_channel_init_pdm_rx_mode(s_mic_rx, &pdm_cfg), TAG, "mic pdm");

    static const audio_codec_data_if_t mic_if = { .enable = mic_enable, .read = mic_read };
    esp_codec_dev_cfg_t in_cfg = { .dev_type = ESP_CODEC_DEV_TYPE_IN, .data_if = &mic_if };
    *mic = s_mic = esp_codec_dev_new(&in_cfg);
    return *mic ? ESP_OK : ESP_FAIL;
}

/*
 * PWR, through the AXP192's PEK key: a tap toggles recording (tap to talk,
 * tap to send; a lone tap auto-sends after 15 s). Hold-to-talk is impossible:
 * the PEK short/long IRQs only fire on release, and the edge IRQs never fire
 * at all. Power off comes from the settings menu (or a 10 s hold, which the
 * PMU cuts in hardware). The touchscreen runs the menus.
 */
static unsigned poll_buttons(void)
{
    static unsigned tick;
    uint8_t st = 0;
    if (s_pmu && tick % PMU_KEY_EVERY == 0 &&
        axp_read(AXP_REG_INTSTS3, &st, 1) == ESP_OK) {
        st &= AXP192_PEK_SHORT | AXP192_PEK_LONG;
        if (st) {
            axp_write(AXP_REG_INTSTS3, st);
        }
    }
    /* Decoded every poll so the auto-release fires on time. */
    unsigned out = 0;
    unsigned ev = axp192_pek_poll(&s_pek, st, tick);
    tick++;
    if (ev & AXP192_PEK_PRESS) {
        out |= MUSE_BTN_TALK_PRESS;
    }
    if (ev & AXP192_PEK_RELEASE) {
        out |= MUSE_BTN_TALK_RELEASE;
    }
    return out;
}

/* Battery voltage and state from the AXP192's ADC; the level curve is
 * M5Unified's (AXP192_Class::getBatteryLevel). */
static esp_err_t read_power(muse_power_t *out)
{
    if (!s_pmu) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t status = 0;
    uint8_t batt[4] = { 0, 0, 0, 0 };
    ESP_RETURN_ON_ERROR(axp_read(AXP_REG_STATUS, &status, 1), TAG, "pmu status");
    ESP_RETURN_ON_ERROR(axp_read(AXP_REG_BATT_V, batt, sizeof(batt)), TAG, "battery adc");
    out->usb = (status & 0x20) != 0;
    out->charging = (status & 0x04) != 0;
    int raw = (batt[0] << 4) + batt[1];
    out->battery_mv = (int)(raw * 1.1f);
    int current = (batt[2] << 5) + batt[3];
    int pct = raw > 3150 ? (int)((raw - 3075) * 0.16f) : raw > 2690 ? (int)((raw - 2690) * 0.027f) : 0;
    if (current > 16) {
        pct -= 16;
    }
    out->battery_pct = pct < 0 ? 0 : pct > 100 ? 100 : pct;
    return ESP_OK;
}

static esp_err_t power_off(void)
{
    ESP_RETURN_ON_FALSE(s_pmu, ESP_ERR_INVALID_STATE, TAG, "no PMU");
    ESP_RETURN_ON_ERROR(bsp_display_backlight_off(), TAG, "backlight off");
    bsp_display_lock(0);
    esp_err_t err = lvgl_port_stop();
    bsp_display_unlock();
    ESP_RETURN_ON_ERROR(err, TAG, "display stop");
    if (s_spk) {
        esp_codec_dev_close(s_spk);
    }
    if (s_mic) {
        esp_codec_dev_close(s_mic);
    }
    /* PWR turns it back on. */
    return axp_write(AXP_REG_OFF_CTL, 0x80);
}

static const muse_board_t s_board = {
    .name = "M5Stack Core2",
    .width = BSP_LCD_H_RES,
    .height = BSP_LCD_V_RES,
    .round = false,
    .touch = true,
    .diagonal_in = 2.0f,
    .talk_button = "power",
    .talk_hint = { LV_ALIGN_TOP_LEFT, 8, 8 },    /* PWR, on the left edge */
    .frame_ms = 40,
    .init = init,
    .display_start = display_start,
    .display_lock = display_lock,
    .display_unlock = bsp_display_unlock,
    .set_brightness = set_brightness,
    .audio_init = audio_init,
    .mic_slot = -1,           /* mix both slots; the wired one is undocumented */
    .set_mic_gain = set_mic_gain,
    .poll_buttons = poll_buttons,
    .read_power = read_power,
    .power_off = power_off,
};

/* Home Link's app_main starts Muse with this board (main/main.c). */
const muse_board_t *muse_board_get(void)
{
    return &s_board;
}
