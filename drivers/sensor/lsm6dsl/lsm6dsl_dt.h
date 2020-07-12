/*
 * Copyright (c) 2020 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LSM6DSL_SPI_CS_CONFIG(n)                                               \
	static struct spi_cs_control lsm6dsx_cs_ctrl_##n = {                   \
		.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(n),                   \
		.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(n),            \
	}

#define LSM6DSL_SPI_CS_REF(n)                                                  \
	COND_CODE_1(DT_INST_SPI_DEV_HAS_CS_GPIOS(n), &(lsm6dsx_cs_ctrl_##n),   \
		    (NULL))

#define LSM6DSL_SPI_CONFIG(n)                                                  \
	COND_CODE_1(DT_INST_SPI_DEV_HAS_CS_GPIOS(n),                           \
		    (LSM6DSL_SPI_CS_CONFIG(n)), ());                           \
	static const struct spi_config lsm6dsx_spi_config_##n = {              \
		.frequency = DT_INST_PROP(n, spi_max_frequency),               \
		.operation =                                                   \
			(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA |  \
			 SPI_WORD_SET(8) | SPI_LINES_SINGLE),                  \
		.slave = DT_INST_REG_ADDR(n),                                  \
		.cs = LSM6DSL_SPI_CS_REF(n),                                   \
	}

#define LSM6DSL_BUS_CONFIG(n)                                                  \
	COND_CODE_1(DT_INST_ON_BUS(n, spi), (LSM6DSL_SPI_CONFIG(n)), ())

#define LSM6DSL_SPI_GPIO_NAME(n)                                               \
	COND_CODE_1(DT_INST_SPI_DEV_HAS_CS_GPIOS(n),                           \
		    (DT_INST_SPI_DEV_CS_GPIOS_LABEL(n)), (NULL))

#define LSM6DSL_SPI_INIT(n)                                                    \
	.comm_init = lsm6dsl_spi_init,                                         \
	.spi.spi_conf = &(lsm6dsx_spi_config_##n),                             \
	.spi.gpio_name = LSM6DSL_SPI_GPIO_NAME(n)

#define LSM6DSL_I2C_INIT(n)                                                    \
	.comm_init = lsm6dsl_i2c_init, .i2c.dev_addr = DT_INST_REG_ADDR(n)

#define LSM6DSL_BUS_INIT(n)                                                    \
	COND_CODE_1(DT_INST_ON_BUS(n, spi), (LSM6DSL_SPI_INIT(n)),             \
		    (LSM6DSL_I2C_INIT(n)))

#if defined(CONFIG_LSM6DSL_TRIGGER)
#define LSM6DSL_TRIGGER_INIT(n)                                                \
	.trigger_name = DT_INST_GPIO_LABEL(n, irq_gpios),                      \
	.trigger_pin = DT_INST_GPIO_PIN(n, irq_gpios),                         \
	.trigger_flags = DT_INST_GPIO_FLAGS(n, irq_gpios),
#else
#define LSM6DSL_TRIGGER_INIT(n)
#endif

#define LSM6DSL_DEVICE(n)                                                      \
	static struct lsm6dsl_data lsm6dsx_data_##n;                           \
	LSM6DSL_BUS_CONFIG(n);                                                 \
	static const struct lsm6dsl_config lsm6dsx_config_##n = {              \
		.comm_master_dev_name = DT_INST_BUS_LABEL(n),                  \
		.wai = UTIL_CAT(DT_DRV_COMPAT, _WHO_AM_I),                     \
		LSM6DSL_BUS_INIT(n),                                           \
		LSM6DSL_TRIGGER_INIT(n)                                        \
	};                                                                     \
	DEVICE_AND_API_INIT(lsm6dsx_##n, DT_INST_LABEL(n), lsm6dsl_init,       \
			    &lsm6dsx_data_##n, &lsm6dsx_config_##n,            \
			    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,          \
			    &lsm6dsl_api_funcs)

DT_INST_FOREACH_STATUS_OKAY(LSM6DSL_DEVICE);
