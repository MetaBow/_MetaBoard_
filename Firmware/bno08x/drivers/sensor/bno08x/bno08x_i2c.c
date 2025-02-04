/*
 * Copyright (c) 2024 Diodes Delight
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for BNO08Xs accessed via I2C.
 */

#include "bno08x.h"

static int bno08x_bus_check_i2c(const union bno08x_bus *bus)
{
	return device_is_ready(bus->i2c.bus) ? 0 : -ENODEV;
}

static int bno08x_reg_read_i2c(const union bno08x_bus *bus,
			       uint8_t start, uint8_t *data, uint16_t len)
{
	return i2c_burst_read_dt(&bus->i2c, start, data, len);
}

static int bno08x_reg_write_i2c(const union bno08x_bus *bus, uint8_t start,
				const uint8_t *data, uint16_t len)
{
	return i2c_burst_write_dt(&bus->i2c, start, data, len);
}

static int bno08x_bus_init_i2c(const union bno08x_bus *bus)
{
	/* I2C is used by default
	 */
	return 0;
}

const struct bno08x_bus_io bno08x_bus_io_i2c = {
	.check = bno08x_bus_check_i2c,
	.read = bno08x_reg_read_i2c,
	.write = bno08x_reg_write_i2c,
	.init = bno08x_bus_init_i2c,
};
