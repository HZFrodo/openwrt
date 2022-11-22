// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 Furong Xu <xfr@outlook.com>

#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/mutex.h>
#include <linux/led-class-multicolor.h>

/* encoded byte tables
 * we present each bit as 3 bits (oversampling by a factor of 3)
 *   zero is represented as 0b100
 *   one is represented as 0b110
 * so each byte is actually represented as 3 bytes
 *
 * the following are tables to avoid repeated computations
 * and are sent via spi as high, medium, low at 3* required HZ
 */
const char byte2encoding_h[/* (value >> 5) & 0x07 */] = {
	0x92,
	0x93,
	0x9a,
	0x9b,
	0xd2,
	0xd3,
	0xda,
	0xdb
};

const char byte2encoding_m[/* (value >> 3) & 0x03 */] = {
	0x49,
	0x4d,
	0x69,
	0x6d
};

const char byte2encoding_l[/* value & 0x07 */] = {
	0x24,
	0x26,
	0x34,
	0x36,
	0xa4,
	0xa6,
	0xb4,
	0xb6
};

struct ws2812b_encoding {
	u8 h, m, l;
};

struct ws2812b_pixel {
	struct ws2812b_encoding r, g, b;
};

struct spi_ws2812b_chipdef {
	unsigned char chip_num;
	unsigned char led_ctrl_cmd_len;
	unsigned char led_num_channels;
};

static struct spi_ws2812b_chipdef miwifi_ws2812b_spi_led = {
	.chip_num = 2, /* Redmi AX6000 has 2 LED chips */
	.led_num_channels = 3,
	.led_ctrl_cmd_len = 9, /* 9 bytes per LED chip */
};

struct spi_ws2812b_led {
	struct led_classdev_mc mc_ldev;
	struct spi_device *spi;
	struct mutex mutex;
	const struct spi_ws2812b_chipdef *cdef;
	unsigned char *led_ctrl_cmd;
	unsigned char chip_id;
};

static inline void ws2812b_set_encoded_pixel(struct ws2812b_encoding *enc, u8 val)
{
	enc->h = byte2encoding_h[(val >> 5) & 0x07];
	enc->m = byte2encoding_m[(val >> 3) & 0x03];
	enc->l = byte2encoding_l[(val >> 0) & 0x07];
}

static void ws2812b_set_pixel_value(struct ws2812b_pixel *spix, unsigned char r, unsigned char g, unsigned char b)
{
	/* assign the encoded values */
	ws2812b_set_encoded_pixel(&spix->g, g);
	ws2812b_set_encoded_pixel(&spix->r, r);
	ws2812b_set_encoded_pixel(&spix->b, b);
}

static const struct spi_device_id spi_ids[] = {
	{ .name = "ws2812b-spi", },
	{},
};

static const struct of_device_id spi_ws2812b_dt_ids[] = {
	{ .compatible = "xiaomi,ws2812b-spi", .data = &miwifi_ws2812b_spi_led, },
	{},
};

MODULE_DEVICE_TABLE(of, spi_single_wire_dt_ids);

static int ws2812b_rgb_led_brightness_set_blocking(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct led_classdev_mc *mc_ldev = lcdev_to_mccdev(cdev);
	struct spi_ws2812b_led *led = container_of(mc_ldev, struct spi_ws2812b_led, mc_ldev);
	struct ws2812b_pixel spix;
	unsigned char *ctrl_cmd_head;
	unsigned char *ctrl_cmd_payload;
	unsigned char ctrl_cmd_len = led->cdef->chip_num * led->cdef->led_ctrl_cmd_len;
	int ret = 0;

	ctrl_cmd_head = kzalloc(ctrl_cmd_len + 2, GFP_KERNEL); /* 2 bytes pad to make waveform more stable */
	if (!ctrl_cmd_head)
		return -ENOMEM;

	mutex_lock(&led->mutex);

	led_mc_calc_color_components(mc_ldev, brightness);
	ws2812b_set_pixel_value(&spix, mc_ldev->subled_info[0].brightness,
		mc_ldev->subled_info[1].brightness, mc_ldev->subled_info[2].brightness);

	ctrl_cmd_payload = ctrl_cmd_head + 1; /* skip the SOF byte */
	memcpy(led->led_ctrl_cmd, &spix, sizeof(spix)); /* save the current color */

	/* now we output the color data via spi */
	memcpy(ctrl_cmd_payload, led->led_ctrl_cmd - led->cdef->led_ctrl_cmd_len * led->chip_id,
		led->cdef->led_ctrl_cmd_len * led->cdef->chip_num);
	ret = spi_write(led->spi, ctrl_cmd_head, ctrl_cmd_len + 2); /* 2 bytes pad */

	mutex_unlock(&led->mutex);

	kfree(ctrl_cmd_head);

	return ret;
}

static int spi_ws2812b_probe(struct spi_device *spi)
{
	struct spi_ws2812b_led *led;
	const struct spi_ws2812b_chipdef *cdef;
	struct device *dev = &spi->dev;
	struct device_node *np = dev_of_node(dev);
	struct device_node *chip_node;
	unsigned char *chip_cmd_buffer;
	const char *state;
	unsigned char cur_chip_id = 0;
	int ret = 0;
	int chip_count;

	struct led_classdev *cdev;

	struct led_init_data init_data = {};

	cdef = device_get_match_data(dev);
	if (!cdef)
		return -ENODEV;

	chip_count = of_get_available_child_count(np);
	dev_info(dev, "Device has %d LED chip(s)\n", chip_count);
	if (chip_count == 0 || chip_count != cdef->chip_num)
		return -ENODEV;

	chip_cmd_buffer = devm_kzalloc(dev, cdef->led_ctrl_cmd_len * cdef->chip_num, GFP_KERNEL);
	if (!chip_cmd_buffer)
		return -ENOMEM;

	for_each_available_child_of_node(np, chip_node) {
		led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
		if (!led)
			return -ENOMEM;

		/* all LEDs of one chip share one cmd_buffer */
		led->led_ctrl_cmd = chip_cmd_buffer + cdef->led_ctrl_cmd_len * cur_chip_id;

		led->spi = spi;
		mutex_init(&led->mutex);
		led->cdef = cdef;
		led->chip_id = cur_chip_id;

		led->mc_ldev.num_colors = cdef->led_num_channels;
		init_data.fwnode = &chip_node->fwnode;

		led->mc_ldev.subled_info = devm_kzalloc(dev, sizeof(struct mc_subled) * led->mc_ldev.num_colors, GFP_KERNEL);
		if (!led->mc_ldev.subled_info)
			return -ENOMEM;

		led->mc_ldev.subled_info[0].color_index = LED_COLOR_ID_RED;
		led->mc_ldev.subled_info[0].channel = 0;
		led->mc_ldev.subled_info[1].color_index = LED_COLOR_ID_GREEN;
		led->mc_ldev.subled_info[1].channel = 1;
		led->mc_ldev.subled_info[2].color_index = LED_COLOR_ID_BLUE;
		led->mc_ldev.subled_info[2].channel = 2;

		cdev = &led->mc_ldev.led_cdev;
		cdev->brightness = LED_OFF;
		cdev->max_brightness = LED_FULL;
		cdev->brightness_set_blocking = ws2812b_rgb_led_brightness_set_blocking;

		state = of_get_property(chip_node, "default-state", NULL);
		if (state) {
			if (!strcmp(state, "on"))
				cdev->brightness = cdev->max_brightness;
			/* default to LED_OFF already */
		}
		ws2812b_rgb_led_brightness_set_blocking(&led->mc_ldev.led_cdev, cdev->brightness);

		spi_set_drvdata(spi, led);

		if ((ret = devm_led_classdev_multicolor_register_ext(&spi->dev, &led->mc_ldev, &init_data)) < 0) {
			dev_err(dev, "Cannot register LED %pOF: %i\n", chip_node, ret);
			return -ENOMEM;
		} else {
			dev_info(dev, "register LED %pOF OK\n", chip_node);
		}

		cur_chip_id++;
	}

	return 0;
}

static struct spi_driver spi_ws2812b_led_driver = {
	.probe		= spi_ws2812b_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= spi_ws2812b_dt_ids,
	},
	.id_table = spi_ids,
};

module_spi_driver(spi_ws2812b_led_driver);

MODULE_AUTHOR("Furong Xu <xfr@outlook.com>");
MODULE_DESCRIPTION("WS2812B RGB LED driver via SPI");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:leds-spi-ws2812b");
