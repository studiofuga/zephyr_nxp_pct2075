/*
 * Copyright (c) 2019 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_pct2075

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>


#include <zephyr/logging/log.h>

#include "pct2075.h"

LOG_MODULE_REGISTER(PCT2075, CONFIG_SENSOR_LOG_LEVEL);

#define PCT2075_REG_TEMP 		0x00
#define PCT2075_REG_CONFIG 		0x01
#define PCT2075_REG_HYST 		0x02
#define PCT2075_REG_OVERTEMP 	0x03
#define PCT2075_REG_TIDLE 		0x04

#define PCT2075_TEMP_SHUTDOWN 	0x01

#define TYPE_UNKNOWN 0x00
#define TYPE_STD 0x01
#define TYPE_P110 0x2
#define TYPE_N005 0x03

#define OVR_STD 0x5000
#define OVR_P110 0x6e00
#define OVR_N005 0xfb00

#define HYS_STD 0x4b00
#define HYS_P110 0x6900
#define HYS_N005 0xf600

struct pct2075_data {
	uint16_t addr;
	uint8_t type;

	uint16_t rawtemp;
};

struct pct2075_config {
	struct i2c_dt_spec i2c;
};

static int read_register(const struct device *dev, uint8_t reg, void *dt, uint8_t dtlen)
{
    const struct pct2075_config *config = dev->config;

	uint8_t cmd[] = {reg};
	int ret = i2c_write_read_dt(&config->i2c, cmd, sizeof(cmd), dt, dtlen);
	if (ret)
	{
		LOG_ERR("Failed to read register! (err %i)", ret);
		return -EIO;
	}

	return 0;
}

static int read_uint16_register(const struct device *dev, uint8_t reg, uint16_t *v)
{
	uint8_t rv[2];
	int err;

	err = read_register(dev, reg, rv, 2);
	if (err) return err;

	*v = sys_be16_to_cpu(*rv);
	return 0;
}

static int pct2075_chip_init(const struct device *dev)
{
    const struct pct2075_config *config = dev->config;
    struct pct2075_data *data = dev->data;

    /* Get the SPI device */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("Bus device is not ready");
        return -ENODEV;
    }

    uint8_t conf;
    if (read_register(dev, PCT2075_REG_CONFIG, &conf, 1)) {
    	LOG_ERR("Cannot read config register.");
    	return -EIO;
    }

	uint16_t tos, thys;

    if (read_uint16_register(dev, PCT2075_REG_OVERTEMP, &tos)) {
    	LOG_ERR("Cannot read overtemp register.");
    	return -EIO;
    }

    if (read_uint16_register(dev, PCT2075_REG_HYST, &thys)) {
    	LOG_ERR("Cannot read hysteresis register.");
    	return -EIO;
    }

    if (tos == OVR_STD && thys == HYS_STD) {
    	data->type = TYPE_STD;
    	LOG_INF("pct2075 Standard");
    } else if (tos == OVR_P110 && thys == HYS_P110) {
    	data->type = TYPE_P110;
    	LOG_INF("pct2075 P110");
	} else if (tos == OVR_N005 && thys == HYS_N005) {
    	data->type = TYPE_N005;
    	LOG_INF("pct2075 N005");
	} else {
    	data->type = TYPE_UNKNOWN;		
		LOG_INF("pct2075 Unknown: %04x %04x", tos, thys);
	}

	return 0;
}

static int pct2075_sample_fetch(const struct device *dev,
			       enum sensor_channel chan)
{
    struct pct2075_data *data = dev->data;

	if (read_uint16_register(dev, PCT2075_REG_TEMP, &data->rawtemp)) {
    	LOG_ERR("Cannot read temperature register.");
    	return -EIO;
    }

    data->rawtemp = data->rawtemp >> 5;

	return 0;
}

static int pct2075_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

    struct pct2075_data *data = dev->data;
    val->val1 = data->rawtemp >> 3;
    val->val2 = (data->rawtemp & 0x07) * 125000;

	return 0;
}

static const struct sensor_driver_api pct2075_api_funcs = {
	.sample_fetch = pct2075_sample_fetch,
	.channel_get = pct2075_channel_get,
};


/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define PCT2075_DEFINE(inst)						\
	static struct pct2075_data pct2075_data_##inst;			\
	static const struct pct2075_config pct2075_config_##inst = { \
		.i2c = I2C_DT_SPEC_INST_GET(inst),\
	}; \
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			 pct2075_chip_init,				\
			 PM_DEVICE_DT_INST_GET(inst),			\
			 &pct2075_data_##inst,				\
			 &pct2075_config_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &pct2075_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(PCT2075_DEFINE)
