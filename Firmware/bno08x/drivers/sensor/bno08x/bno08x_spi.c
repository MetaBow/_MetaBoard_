/*
 * Copyright (c) 2024 Diodes Delight
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for BNO08Xs accessed via SPI.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "bno08x.h"

LOG_MODULE_DECLARE(bno08x, CONFIG_SENSOR_LOG_LEVEL);

static int bno08x_bus_check_spi(const union bno08x_bus *bus)
{
	return spi_is_ready_dt(&bus->spi) ? 0 : -ENODEV;
}

static int bno08x_reg_read_spi(const union bno08x_bus *bus,
			       uint8_t reg, uint8_t *data, uint16_t len)
{
	int ret;
	// LOG_ERR("bno08x_reg_read_spi");
	const struct spi_buf tx_buf = {
		.buf = &reg,
		.len = 1
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf = {
		.buf = data,
		.len = len
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};
	
	ret = spi_transceive_dt(&bus->spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("spi_transceive failed %i", ret);
		return ret;
	}

	return 0;
}

static int bno08x_reg_write_spi(const union bno08x_bus *bus, const uint8_t *data, uint16_t len)
{
	int ret;
	// LOG_ERR("bno08x_reg_write_spi");
	const struct spi_buf tx_buf = {
		.buf = data,
		.len = len
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	ret = spi_write_dt(&bus->spi, &tx);
	if (ret < 0) {
		LOG_ERR("spi_write_dt failed %i", ret);
		return ret;
	}

	return 0;
}

static int bno08x_bus_init_spi(const union bno08x_bus *bus)
{
	//assuming strapping pin set in hardware for now
	return 0;
}

const struct bno08x_bus_io bno08x_bus_io_spi = {
	.check = bno08x_bus_check_spi,
	.read = bno08x_reg_read_spi,
	.write = bno08x_reg_write_spi,
	.init = bno08x_bus_init_spi,
};
