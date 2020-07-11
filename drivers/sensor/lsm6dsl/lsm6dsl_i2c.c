/* lsm6dsl_i2c.c - I2C routines for LSM6DSL driver
 */

/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "lsm6dsl.h"

static int lsm6dsl_i2c_read_data(struct device *dev, uint8_t reg_addr,
				 uint8_t *value, uint8_t len)
{
	struct lsm6dsl_data *data = dev->driver_data;
	const struct lsm6dsl_config * const config = dev->config_info;

	return i2c_burst_read(data->comm_master, config->i2c.dev_addr,
			      reg_addr, value, len);
}

static int lsm6dsl_i2c_write_data(struct device *dev, uint8_t reg_addr,
				  uint8_t *value, uint8_t len)
{
	struct lsm6dsl_data *data = dev->driver_data;
	const struct lsm6dsl_config * const config = dev->config_info;

	return i2c_burst_write(data->comm_master, config->i2c.dev_addr,
			       reg_addr, value, len);
}

static int lsm6dsl_i2c_read_reg(struct device *dev, uint8_t reg_addr,
				uint8_t *value)
{
	struct lsm6dsl_data *data = dev->driver_data;
	const struct lsm6dsl_config * const config = dev->config_info;

	return i2c_reg_read_byte(data->comm_master, config->i2c.dev_addr,
				 reg_addr, value);
}

static int lsm6dsl_i2c_update_reg(struct device *dev, uint8_t reg_addr,
				  uint8_t mask, uint8_t value)
{
	struct lsm6dsl_data *data = dev->driver_data;
	const struct lsm6dsl_config * const config = dev->config_info;

	return i2c_reg_update_byte(data->comm_master, config->i2c.dev_addr,
				   reg_addr, mask, value);
}

static const struct lsm6dsl_transfer_function lsm6dsl_i2c_transfer_fn = {
	.read_data = lsm6dsl_i2c_read_data,
	.write_data = lsm6dsl_i2c_write_data,
	.read_reg  = lsm6dsl_i2c_read_reg,
	.update_reg = lsm6dsl_i2c_update_reg,
};

int lsm6dsl_i2c_init(struct device *dev)
{
	struct lsm6dsl_data *data = dev->driver_data;

	data->hw_tf = &lsm6dsl_i2c_transfer_fn;

	return 0;
}
