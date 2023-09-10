/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2019, Nordic Semiconductor ASA
 * Copyright (c) 2021 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_pwm

#include <drivers/led_strip.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(ws2812_pwm);

#include <zephyr.h>
#include <device.h>
#include <drivers/pwm.h>
#include <sys/math_extras.h>
#include <sys/util.h>
#include <dt-bindings/led/led.h>
#include <nrfx_pwm.h>

/* pwm-one-frame and pwm-zero-frame in DT are for 8-bit frames. */
#define PWM_FRAME_BITS 16
#define PWM_NRFX_CH_POLARITY_MASK BIT(15)
#define PWM_NRFX_CH_VALUE(value, inverted) (value | (inverted ? 0 : PWM_NRFX_CH_POLARITY_MASK))

/*
 * PWM master configuration:
 *
 * - mode 0 (the default), 8 bit, MSB first (arbitrary), one-line PWM
 */

struct ws2812_nrfx_pwm_cfg {
	nrfx_pwm_t pwm;
	nrfx_pwm_config_t config;
	uint16_t one_frame;
	uint16_t zero_frame;
    bool inverted;
	uint8_t num_colors;
	const uint8_t *color_mapping;
	uint32_t num_leds;
};

struct ws2812_nrfx_pwm_data {
    uint16_t * buffer;
    uint32_t buffer_length;
	volatile nrfx_drv_state_t drv_state;
};

static const struct ws2812_nrfx_pwm_cfg *dev_cfg(const struct device *dev)
{
	return dev->config;
}

static struct ws2812_nrfx_pwm_data *dev_data(const struct device *dev)
{
	return (struct ws2812_nrfx_pwm_data *)dev->data;
}

void ws2811_nrfx_pwm_handler(nrfx_pwm_evt_type_t pwm_evt, void *cb)
{
	if (pwm_evt == NRFX_PWM_EVT_STOPPED) {
		((struct ws2812_nrfx_pwm_data *const)cb)->drv_state = NRFX_DRV_STATE_INITIALIZED;
	}
}

/*
 * Serialize an 8-bit color channel value into an equivalent sequence
 * of PWM frames, MSbit first, where a one bit becomes PWM frame
 * one_frame, and zero bit becomes zero_frame.
 */
static inline void ws2812_nrfx_pwm_ser(uint16_t buf[8], uint8_t color, const uint16_t one_frame,
				       const uint16_t zero_frame)
{
	int i;

	for (i = 0; i < 8; i++) {
		buf[i] = color & BIT(7 - i) ? one_frame : zero_frame;
	}
}

/*
 * Returns true if and only if cfg->px_buf is big enough to convert
 * num_pixels RGB color values into PWM frames.
 */
static inline bool num_pixels_ok(const struct ws2812_nrfx_pwm_cfg *cfg,
				 const struct ws2812_nrfx_pwm_data *data, size_t num_pixels)
{
	size_t nbytes;
	bool overflow;

	overflow = size_mul_overflow(num_pixels, cfg->num_colors * 8, &nbytes);
	return !overflow && (nbytes + 1 <= data->buffer_length);
}

static int ws2812_strip_update_rgb(const struct device *dev, struct led_rgb *pixels,
				   size_t num_pixels)
{
	const struct ws2812_nrfx_pwm_cfg *cfg = dev_cfg(dev);
	struct ws2812_nrfx_pwm_data *data = dev_data(dev);

	// Wait until we are done outputing data from PWM.
	while (data->drv_state == NRFX_DRV_STATE_POWERED_ON) {
		k_usleep(50);
	}

	if (!num_pixels_ok(cfg, data, num_pixels)) {
		return -ENOMEM;
	}

	uint16_t *px_buf = data->buffer;

	/*
	 * Convert pixel data into PWM frames. Each frame has pixel data
	 * in color mapping on-wire format (e.g. GRB, GRBW, RGB, etc).
	 */
	for (int i = 0; i < num_pixels; i++) {
		for (uint8_t j = 0; j < cfg->num_colors; j++) {
			uint8_t pixel;

			switch (cfg->color_mapping[j]) {
			/* White channel is not supported by LED strip API. */
			case LED_COLOR_ID_WHITE:
				pixel = 0;
				break;
			case LED_COLOR_ID_RED:
				pixel = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				pixel = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				pixel = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
			ws2812_nrfx_pwm_ser(px_buf, pixel, cfg->one_frame, cfg->zero_frame);
			px_buf += 8;
		}
	}

	/*
	 * Display the pixel data.
	 */
	data->drv_state = NRFX_DRV_STATE_POWERED_ON;
	nrf_pwm_sequence_t seq = 
    {
        .length = data->buffer_length, 
        .values.p_common = data->buffer
    };
	return nrfx_pwm_simple_playback(&cfg->pwm, &seq, 1, NRFX_PWM_FLAG_STOP);
}

static int ws2812_strip_update_channels(const struct device *dev, uint8_t *channels,
					size_t num_channels)
{
	LOG_ERR("update_channels not implemented");
	return -ENOTSUP;
}

static int ws2812_nrfx_pwm_init(const struct device *dev)
{
	const struct ws2812_nrfx_pwm_cfg *cfg = dev_cfg(dev);
	struct ws2812_nrfx_pwm_data *data = dev_data(dev);
	uint8_t i;

	if (data->drv_state != NRFX_DRV_STATE_UNINITIALIZED) {
		LOG_ERR("RGB strip %s is already initialized", dev->name);
		return -EBUSY;
	}

	for (i = 0; i < cfg->num_colors; i++) {
		switch (cfg->color_mapping[i]) {
		case LED_COLOR_ID_WHITE:
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: invalid channel to color mapping."
				"Check the color-mapping DT property",
				dev->name);
			return -EINVAL;
		}
	}

    IRQ_DIRECT_CONNECT(PWM0_IRQn, 0, nrfx_pwm_0_irq_handler, 0);

	int err_code =
		nrfx_pwm_init(&cfg->pwm, &cfg->config, ws2811_nrfx_pwm_handler, (void *)data);
	if (err_code != NRFX_SUCCESS) {
		return err_code;
	}
    
    //NRF_P0->PIN_CNF[cfg->config.output_pins[0]] |= GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos;

	data->drv_state = NRFX_DRV_STATE_INITIALIZED;

	memset(data->buffer, 0, data->buffer_length);
	data->buffer[data->buffer_length - 1] = PWM_NRFX_CH_VALUE(0, cfg->inverted);

	LOG_DBG("WS2812 PWM driver %s initialized", dev->name);
	return 0;
}

static const struct led_strip_driver_api ws2812_nrfx_pwm_api = {
	.update_rgb = ws2812_strip_update_rgb,
	.update_channels = ws2812_strip_update_channels,
};

#define PWM_SEQ_VALUES_LEN(num_leds) (BITS_TOTAL(num_leds) + PWM_SEQ_VAL_RESET_SLOT)

#define WS2812_PWM_NUM_PIXELS(idx) (DT_INST_PROP(idx, chain_length))
#define WS2812_PWM_HAS_WHITE(idx) (DT_INST_PROP(idx, has_white_channel) == 1)
#define WS2812_PWM_INVERTED(idx) (DT_INST_PROP(idx, pwm_inverted))
#define WS2812_PWM_ONE_FRAME(idx) (DT_INST_PROP(idx, pwm_one_frame))
#define WS2812_PWM_ZERO_FRAME(idx) (DT_INST_PROP(idx, pwm_zero_frame))
#define WS2812_PWM_OUT_PIN(idx)    (DT_INST_PROP(idx, out_pin))
#define WS2812_PWM_OUT_INVERTED(idx)    (DT_INST_PROP(idx, out_inverted) ? NRFX_PWM_PIN_INVERTED : 0)
/* 16 kbits per color per pixel + 1 bit sequence stop */
#define WS2812_PWM_BUFSZ(idx) ((WS2812_NUM_COLORS(idx) * 8 * WS2812_PWM_NUM_PIXELS(idx)) + 1)


#define WS2812_PWM_NODE(idx) DT_INST_PROP(idx, pwm_instance)
#define WS2812_PWM_NODE_LABEL(idx) DT_STRING_TOKEN(WS2812_PWM_NODE(idx), label)

#define WS2812_PWM_NODE_IDX_PWM_0 0
#define WS2812_PWM_NODE_IDX_PWM_1 1
#define WS2812_PWM_NODE_IDX_PWM_2 2
#define WS2812_PWM_NODE_IDX_PWM_3 3
#define WS2812_PWM_NODE_IDX_3(idx) WS2812_PWM_NODE_IDX_ ## idx
#define WS2812_PWM_NODE_IDX_2(idx) WS2812_PWM_NODE_IDX_3(idx)
#define WS2812_PWM_NODE_IDX_1(idx) WS2812_PWM_NODE_IDX_2(idx)
#define WS2812_PWM_NODE_IDX(idx) WS2812_PWM_NODE_IDX_1(WS2812_PWM_NODE_LABEL(idx))

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)                                                                  \
	static const uint8_t ws2812_nrfx_pwm_##idx##_color_mapping[] =                             \
		DT_INST_PROP(idx, color_mapping)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))


#define WS2812_PWM_DEVICE(idx)                                                                     \
	static uint16_t ws2812_nrfx_pwm_##idx##_px_buf[WS2812_PWM_BUFSZ(idx)];                      \
	WS2812_COLOR_MAPPING(idx);                                                                 \
	NRF_DT_CHECK_PIN_ASSIGNMENTS(WS2812_PWM_NODE(idx), 1);             \
                                                                                                   \
									 \
	static const struct ws2812_nrfx_pwm_cfg ws2812_nrfx_pwm_##idx##_cfg = {	 \
		.pwm = NRFX_PWM_INSTANCE(WS2812_PWM_NODE_IDX(idx)),				      \
		.config = {					      \
            .skip_gpio_cfg = false,			      \
            .skip_psel_cfg = false,					      \
            .output_pins = {			      \
                WS2812_PWM_OUT_PIN(idx),		\
                NRFX_PWM_PIN_NOT_USED,		      \
                NRFX_PWM_PIN_NOT_USED,		      \
                NRFX_PWM_PIN_NOT_USED,		      \
            },					      \
            .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,\
			.base_clock = NRF_PWM_CLK_16MHz,			      \
			.count_mode = NRF_PWM_MODE_UP,		      \
			.top_value = 20UL, /* Compare 20 at 16MHz gives 800kHz output */ \
			.load_mode = NRF_PWM_LOAD_COMMON,		      \
			.step_mode = NRF_PWM_STEP_AUTO,		      \
		},							      \
		.one_frame  = PWM_NRFX_CH_VALUE(WS2812_PWM_ONE_FRAME(idx), WS2812_PWM_INVERTED(idx)),			 \
		.zero_frame = PWM_NRFX_CH_VALUE(WS2812_PWM_ZERO_FRAME(idx), WS2812_PWM_INVERTED(idx)),		 \
        .inverted = WS2812_PWM_INVERTED(idx), \
		.num_colors = WS2812_NUM_COLORS(idx),			 \
		.color_mapping = ws2812_nrfx_pwm_##idx##_color_mapping,	 \
        .num_leds = WS2812_PWM_NUM_PIXELS(idx),             \
	};                                        \
                                                                                                   \
	static struct ws2812_nrfx_pwm_data ws2812_nrfx_pwm_##idx##_data = {                  \
		.buffer = ws2812_nrfx_pwm_##idx##_px_buf,                                \
		.buffer_length = WS2812_PWM_BUFSZ(idx),                                               \
		.drv_state = NRFX_DRV_STATE_UNINITIALIZED                                          \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(idx, ws2812_nrfx_pwm_init, NULL, &ws2812_nrfx_pwm_##idx##_data, &ws2812_nrfx_pwm_##idx##_cfg, \
			      POST_KERNEL, CONFIG_LED_STRIP_INIT_PRIORITY, &ws2812_nrfx_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_DEVICE)
