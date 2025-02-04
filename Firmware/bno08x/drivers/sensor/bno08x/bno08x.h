/*
 * Copyright (c) 2024 Diodes Delight
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BNO08X_H_
#define ZEPHYR_DRIVERS_SENSOR_BNO08X_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/time_units.h>

#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_err.h"


#define BNO08X_SET_BITS(reg_data, bitname, data)		  \
	((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) \
					  & bitname##_MSK))
#define BNO08X_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

enum bno08x_channel{
 
    /** Quaternion */
    SENSOR_CHAN_ROTATION_VEC_I = SENSOR_CHAN_PRIV_START,
	SENSOR_CHAN_ROTATION_VEC_J,
	SENSOR_CHAN_ROTATION_VEC_K,
	SENSOR_CHAN_ROTATION_VEC_IJKR,
	SENSOR_CHAN_ROTATION_VEC_REAL,
	SENSOR_CHAN_ROTATION_VEC_ACCURACY,
 
};

struct bno08x_data {
	sh2_SensorValue_t sensor_value;
	int16_t ax, ay, az, gx, gy, gz;
	uint8_t acc_range, acc_odr, gyr_odr;
	uint16_t gyr_range;
};

union bno08x_bus {
#if CONFIG_BNO08X_BUS_SPI
	struct spi_dt_spec spi;
#endif
#if CONFIG_BNO08X_BUS_I2C
	struct i2c_dt_spec i2c;
#endif
};

typedef int (*bno08x_bus_check_fn)(const union bno08x_bus *bus);
typedef int (*bno08x_bus_init_fn)(const union bno08x_bus *bus);
typedef int (*bno08x_reg_read_fn)(const union bno08x_bus *bus,
				  uint8_t reg,
				  uint8_t *data,
				  uint16_t len);
typedef int (*bno08x_reg_write_fn)(const union bno08x_bus *bus,
				   const uint8_t *data,
				   uint16_t len);

struct bno08x_bus_io {
	bno08x_bus_check_fn check;
	bno08x_reg_read_fn read;
	bno08x_reg_write_fn write;
	bno08x_bus_init_fn init;
};

struct bno08x_config {
	union bno08x_bus bus;
	const struct bno08x_bus_io *bus_io;
	struct gpio_dt_spec irq;
	struct gpio_dt_spec wake;
	struct gpio_dt_spec reset;
#if CONFIG_BNO08X_TRIGGER
	struct gpio_dt_spec int1;
	struct gpio_dt_spec int2;
#endif
};

#if CONFIG_BNO08X_BUS_SPI
#define BNO08X_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA)
#define BNO08X_SPI_ACC_DELAY_US 2
extern const struct bno08x_bus_io bno08x_bus_io_spi;
#endif

#if CONFIG_BNO08X_BUS_I2C
extern const struct bno08x_bus_io bno08x_bus_io_i2c;
#endif

int bno08x_reg_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t length);

int bno08x_reg_write(const struct device *dev, const uint8_t *data, uint16_t length);

int bno08x_reg_write_with_delay(const struct device *dev,
				uint8_t reg,
				const uint8_t *data,
				uint16_t length,
				uint32_t delay_us);

#ifdef CONFIG_BNO08X_TRIGGER
int bno08x_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int bno08x_init_interrupts(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_BNO08X_H_ */
