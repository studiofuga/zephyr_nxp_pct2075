/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

static const struct device *get_pct2075_device(void)
{
	const struct device *const dev = DEVICE_DT_GET_ANY(nxp_pct2075);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	const struct device *dev = get_pct2075_device();

	if (dev == NULL) {
		return;
	}

	int numerr = 0;
	while (1) {
		struct sensor_value temp;

		sensor_sample_fetch(dev);
		ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		if (ret) {
			printk("Error: %d\n", ret);
			++numerr;
			if (numerr > 5)
				return;
		} else {
			numerr = 0;
		}

		printk("temperature: %d.%06d\n", temp.val1, temp.val2);

		gpio_pin_set_dt(&led, 1);
		k_sleep(K_MSEC(500));
		gpio_pin_set_dt(&led, 0);

		k_sleep(K_MSEC(500));
	}
}

