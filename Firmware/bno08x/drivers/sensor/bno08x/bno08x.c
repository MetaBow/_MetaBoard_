/*
 * Copyright (c) 2024 Diodes Delight
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ceva_bno08x

#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>


#include "bno08x.h"

#define SAMPLE_INTERVAL_US 2000


sh2_Hal_t sh2_HAL;
sh2_ProductIds_t productIds;

static bool bno08x_wait_for_int(const struct device *dev);
static int sh2_bus_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, const struct device *dev);
static int sh2_bus_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us,const struct device *dev);
static void sh2_bus_close(sh2_Hal_t *self, const struct device *dev);
static int sh2_bus_open(sh2_Hal_t *self, const struct device *dev);

static uint32_t sh2_getTimeUs(sh2_Hal_t *self);
static void sh2_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sh2_sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent, const struct device *dev);
static void bno08x_reset(const struct device *dev);
static void bno08x_wake(const struct device *dev);
static bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us,
							   	   uint32_t sensorSpecific, const struct device *dev);

LOG_MODULE_REGISTER(bno08x, LOG_LEVEL_ERR);

static inline int bno08x_bus_check(const struct device *dev)
{
	const struct bno08x_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static inline int bno08x_bus_init(const struct device *dev)
{
	const struct bno08x_config *cfg = dev->config;

	return cfg->bus_io->init(&cfg->bus);
}

int bno08x_reg_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
	const struct bno08x_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, reg, data, length);
}

int bno08x_buf_write(const struct device *dev,
		     const uint8_t *data, uint16_t length)
{
	const struct bno08x_config *cfg = dev->config;

	return cfg->bus_io->write(&cfg->bus, data, length);
}

static int bno08x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	// todo maybe allow for enabling only needed reports
	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}
	enableReport(SH2_ACCELEROMETER, SAMPLE_INTERVAL_US, 0, dev);
	enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, SAMPLE_INTERVAL_US, 0, dev);
	// enableReport(SH2_LINEAR_ACCELERATION, SAMPLE_INTERVAL_US, 0, dev);
	enableReport(SH2_GYROSCOPE_CALIBRATED, SAMPLE_INTERVAL_US, 0, dev);
	enableReport(SH2_ROTATION_VECTOR, SAMPLE_INTERVAL_US, 0, dev);
	LOG_INF("BNO08X sample fetch");
	sh2_service();

	return 0;
}

static int bno08x_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bno08x_data *data = dev->data;

	switch(chan){
		case SENSOR_CHAN_ACCEL_X:
			sensor_value_from_double(val,data->sensor_value.un.linearAcceleration.x);
			break;
		case SENSOR_CHAN_ACCEL_Y:
			sensor_value_from_double(val,data->sensor_value.un.linearAcceleration.y);
			break;
		case SENSOR_CHAN_ACCEL_Z:
			sensor_value_from_double(val,data->sensor_value.un.linearAcceleration.z);
			break;
		case SENSOR_CHAN_ACCEL_XYZ:
			// sensor_value_from_double(val,data->sensor_value.un.linearAcceleration.x);
			// sensor_value_from_double(val+1,data->sensor_value.un.linearAcceleration.y);
			// sensor_value_from_double(val+2,data->sensor_value.un.linearAcceleration.z);
			val[0] = data->accel[0];
			val[1] = data->accel[1];
			val[2] = data->accel[2];
			break;
		case SENSOR_CHAN_GYRO_X:
			sensor_value_from_double(val,data->sensor_value.un.gyroscope.x);
			break;
		case SENSOR_CHAN_GYRO_Y:
			sensor_value_from_double(val,data->sensor_value.un.gyroscope.y);
			break;
		case SENSOR_CHAN_GYRO_Z:
			sensor_value_from_double(val,data->sensor_value.un.gyroscope.z);
			break;
		case SENSOR_CHAN_GYRO_XYZ:
			// sensor_value_from_double(val,data->sensor_value.un.gyroscope.x);
			// sensor_value_from_double(val+1,data->sensor_value.un.gyroscope.y);
			// sensor_value_from_double(val+2,data->sensor_value.un.gyroscope.z);
			val[0] = data->gyro[0];
			val[1] = data->gyro[1];
			val[2] = data->gyro[2];
			break;
		case SENSOR_CHAN_MAGN_X:
			sensor_value_from_double(val,data->sensor_value.un.magneticField.x);
			break;
		case SENSOR_CHAN_MAGN_Y:	
			sensor_value_from_double(val,data->sensor_value.un.magneticField.y);
			break;
		case SENSOR_CHAN_MAGN_Z:
			sensor_value_from_double(val,data->sensor_value.un.magneticField.z);
			break;
		case SENSOR_CHAN_MAGN_XYZ:
			// sensor_value_from_double(val,data->sensor_value.un.magneticField.x);
			// sensor_value_from_double(val+1,data->sensor_value.un.magneticField.y);
			// sensor_value_from_double(val+2,data->sensor_value.un.magneticField.z);
			val[0] = data->mag[0];
			val[1] = data->mag[1];
			val[2] = data->mag[2];
			break;
		case SENSOR_CHAN_ROTATION_VEC_I:
			sensor_value_from_double(val,data->sensor_value.un.rotationVector.i);
			break;
		case SENSOR_CHAN_ROTATION_VEC_J:
			sensor_value_from_double(val,data->sensor_value.un.rotationVector.j);
			break;
		case SENSOR_CHAN_ROTATION_VEC_K:
			sensor_value_from_double(val,data->sensor_value.un.rotationVector.k);
			break;
		case SENSOR_CHAN_ROTATION_VEC_REAL:
			sensor_value_from_double(val,data->sensor_value.un.rotationVector.real);
			break;
		case SENSOR_CHAN_ROTATION_VEC_IJKR:
			// LOG_INF("SENSOR_CHAN_ROTATION_VEC, I: %f, J: %f, K: %f, R: %f", data->sensor_value.un.rotationVector.i, data->sensor_value.un.rotationVector.j, data->sensor_value.un.rotationVector.k, data->sensor_value.un.rotationVector.real);
			// sensor_value_from_double(val,data->sensor_value.un.rotationVector.i);
			// sensor_value_from_double(val+1,data->sensor_value.un.rotationVector.j);
			// sensor_value_from_double(val+2,data->sensor_value.un.rotationVector.k);
			// sensor_value_from_double(val+3,data->sensor_value.un.rotationVector.real);
			val[0] = data->quat[0];
			val[1] = data->quat[1];
			val[2] = data->quat[2];
			val[3] = data->quat[3];
			break;		
		case SENSOR_CHAN_ROTATION_VEC_ACCURACY:
			sensor_value_from_double(val,data->sensor_value.un.rotationVector.accuracy);
			break;
		default:
			return -ENOTSUP;
	
	}


	return 0;
}

static int bno08x_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	int ret = -ENOTSUP;

	if ((chan == SENSOR_CHAN_ACCEL_X) || (chan == SENSOR_CHAN_ACCEL_Y)
	    || (chan == SENSOR_CHAN_ACCEL_Z)
	    || (chan == SENSOR_CHAN_ACCEL_XYZ)) {
		switch (attr) {
		case SENSOR_ATTR_SAMPLING_FREQUENCY:
			// ret = set_accel_odr_osr(dev, val, NULL);
			break;
		case SENSOR_ATTR_OVERSAMPLING:
			// ret = set_accel_odr_osr(dev, NULL, val);
			break;
		case SENSOR_ATTR_FULL_SCALE:
			// ret = set_accel_range(dev, val);
			break;

		default:
			ret = -ENOTSUP;
		}
	} else if ((chan == SENSOR_CHAN_GYRO_X) || (chan == SENSOR_CHAN_GYRO_Y)
		   || (chan == SENSOR_CHAN_GYRO_Z)
		   || (chan == SENSOR_CHAN_GYRO_XYZ)) {
		switch (attr) {
		case SENSOR_ATTR_SAMPLING_FREQUENCY:
			// ret = set_gyro_odr_osr(dev, val, NULL);
			break;
		case SENSOR_ATTR_OVERSAMPLING:
			// ret = set_gyro_odr_osr(dev, NULL, val);
			break;
		case SENSOR_ATTR_FULL_SCALE:
			// ret = set_gyro_range(dev, val);
			break;
		default:
			ret = -ENOTSUP;
		}
	}

	return ret;
}

static bool bno08x_wait_for_int(const struct device *dev) {

	const struct bno08x_config *cfg = dev->config;

	for (int i = 0; i < 5*10000; i++) {
		
		if (gpio_pin_get_dt(&cfg->irq))
			return 0;

		k_usleep(5);
	}
	LOG_ERR("timed out waiting for interrupt");

	return -ETIMEDOUT;
}

static void sh2_bus_close(sh2_Hal_t *self, const struct device *dev) {
	return;
}

static int sh2_bus_open(sh2_Hal_t *self, const struct device *dev) {
	bno08x_reset(dev);
	bno08x_wait_for_int(dev);
	return 0;
}


static int sh2_bus_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us, const struct device *dev) {

	uint16_t packet_size = 0;
	int ret;
	// LOG_ERR("sh2_bus_read");

	ret = bno08x_wait_for_int(dev);
	if (ret != 0) {
		LOG_ERR("err bno08x_wait_for_int");
		return 0;
	}

	ret = bno08x_reg_read(dev, 0x00, pBuffer, 4);
	if (ret != 0) {
		LOG_ERR("err getting packet size");
		return 0;
	}

	// LOG_HEXDUMP_INF(pBuffer, 4, "pBuffer packet size read");

	// Determine amount to read
	// packet_size = (uint16_t)pBuffer[0] | (uint16_t)pBuffer[1] << 8;
	packet_size = (pBuffer[0] + (pBuffer[1] << 8)) & ~0x8000;
	LOG_INF("packet_size: %d", packet_size);

	if (packet_size > len) {
		LOG_ERR("packet_size larger than expected: %d, requested len: %d", packet_size, len);
		return 0;
	}

	ret = bno08x_wait_for_int(dev);
	if (ret != 0) {
		LOG_ERR("err bno08x_wait_for_int");
		return 0;
	}

	ret = bno08x_reg_read(dev, 0x00, pBuffer, packet_size);
	if (ret != 0) {
		LOG_ERR("err getting data");
		return 0;
	}


	
	LOG_HEXDUMP_INF(pBuffer, packet_size, "pBuffer read");
	// if (!dev->read(pBuffer, packet_size, 0x00)) {
	// 	return 0;
	// }

	return packet_size;

}

static int sh2_bus_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, const struct device *dev) {
	// LOG_ERR("sh2_bus_write");
	int ret;

	ret = bno08x_wait_for_int(dev);
	if (ret != 0) {
		LOG_ERR("sh2_bus_write timeout waiting for interrupt");
		return 0;
	}
	
	ret = bno08x_buf_write(dev, pBuffer, len);
	if (ret != 0) {
		LOG_ERR("sh2_bus_write SPI write error");
		return 0;
	}
	return len;
}

static bool enableReport(sh2_SensorId_t sensorId, uint32_t interval_us,
							   	   uint32_t sensorSpecific, const struct device *dev) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = sensorSpecific;

  config.reportInterval_us = interval_us;

  bno08x_wait_for_int(dev);
  
  int status = sh2_setSensorConfig(sensorId, &config);
  LOG_INF("enableReport: %d", status);

  if (status != SH2_OK) {
	LOG_ERR("Error setting sensor config: %d", status);
    return -ENODATA;
  }

  return 0;
}

static void bno08x_reset(const struct device *dev) {
	const struct bno08x_config *cfg = dev->config;
	LOG_WRN("bno08x_reset");
    gpio_pin_set_dt(&cfg->reset, 0);
    k_msleep(3);
    gpio_pin_set_dt(&cfg->reset, 1);
	k_msleep(3);
	return;
}

static void bno08x_wake(const struct device *dev) {
	const struct bno08x_config *cfg = dev->config;

    gpio_pin_set_dt(&cfg->wake, 0);
    bno08x_wait_for_int(dev);
	k_usleep(50);
    gpio_pin_set_dt(&cfg->wake, 1);
	return;
}

static void sh2_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
	// If we see a reset, set a flag so that sensors will be reconfigured.
	LOG_INF("sh2_callback %d",pEvent->eventId);
	// LOG_ERR("sh2_callback %d",pEvent->shtpEvent);
	if (pEvent->eventId == SH2_RESET) {
		LOG_ERR("SH2_RESET");
	}
}

// Handle sensor events.
/*static void sh2_sensorHandler(void *cookie, sh2_SensorEvent_t *event, const struct device *dev) {
	LOG_WRN("sh2_sensorHandler");
	struct bno08x_data *data = dev->data;
	// static sh2_SensorValue_t _sensor_value;
	// static sh2_SensorValue_t *sensor_value;
	int ret = sh2_decodeSensorEvent(&data->sensor_value, event);
	
	if (ret != SH2_OK) {
		LOG_ERR("Error decoding sensor event: %d", ret);
		data->sensor_value.timestamp = 0;
		return;
	}

}
*/


/* This is the callback from sh2_setSensorCallback() in your code. */
static void sh2_sensorHandler(void *cookie, sh2_SensorEvent_t *event,
                              const struct device *dev)
{
    struct bno08x_data *data = dev->data;

    // 1) Decode into a temporary struct
    sh2_SensorValue_t decoded;
    int rc = sh2_decodeSensorEvent(&decoded, event);
    if (rc != SH2_OK) {
        // LOG_ERR("Error decoding sensor event: %d", rc);
        return;
    }

    // 2) Switch on reportId, then use sensor_value_from_double
    //    to preserve fractional data in val2.
    switch (decoded.sensorId) {
    case SH2_ACCELEROMETER:
        sensor_value_from_double(&data->accel[0], decoded.un.accelerometer.x);
        sensor_value_from_double(&data->accel[1], decoded.un.accelerometer.y);
        sensor_value_from_double(&data->accel[2], decoded.un.accelerometer.z);
        break;

    case SH2_GYROSCOPE_CALIBRATED:
        sensor_value_from_double(&data->gyro[0], decoded.un.gyroscope.x);
        sensor_value_from_double(&data->gyro[1], decoded.un.gyroscope.y);
        sensor_value_from_double(&data->gyro[2], decoded.un.gyroscope.z);
        break;

    case SH2_MAGNETIC_FIELD_CALIBRATED:
        sensor_value_from_double(&data->mag[0], decoded.un.magneticField.x);
        sensor_value_from_double(&data->mag[1], decoded.un.magneticField.y);
        sensor_value_from_double(&data->mag[2], decoded.un.magneticField.z);
        break;

    case SH2_ROTATION_VECTOR:
        // i, j, k, real
        sensor_value_from_double(&data->quat[0], decoded.un.rotationVector.i);
        sensor_value_from_double(&data->quat[1], decoded.un.rotationVector.j);
        sensor_value_from_double(&data->quat[2], decoded.un.rotationVector.k);
        sensor_value_from_double(&data->quat[3], decoded.un.rotationVector.real);
        // If you need 'accuracy', do something similar with
        // decoded.un.rotationVector.accuracy
        break;

    /* handle other sensors you want (Linear Accel, Gravity, etc.) */

    default:
        // For sensors you’re not using, either ignore or log them
        break;
    }
}

static uint32_t sh2_getTimeUs(sh2_Hal_t *self) {
  return k_cyc_to_us_floor32(k_uptime_ticks());
}

static int bno08x_init(const struct device *dev)
{
	int ret;
	struct bno08x_data *data = dev->data;
	const struct bno08x_config *cfg = dev->config;

	sh2_HAL.open = sh2_bus_open;
    sh2_HAL.close = sh2_bus_close;
    sh2_HAL.read = sh2_bus_read;
    sh2_HAL.write = sh2_bus_write;
    sh2_HAL.getTimeUs = sh2_getTimeUs;

    int err;

	LOG_INF("BNO08X init");

	ret = bno08x_bus_check(dev);
	if (ret < 0) {
		LOG_ERR("Could not initialize bus");
		return ret;
	}

	ret = bno08x_bus_init(dev);
	if (ret != 0) {
		LOG_ERR("Could not initiate bus communication");
		return ret;
	}

	if (!device_is_ready(cfg->irq.port)) {
		LOG_DBG("%s not ready", cfg->irq.port->name);
		return -ENODEV;
	}

	if (!device_is_ready(cfg->wake.port)) {
		LOG_DBG("%s not ready", cfg->wake.port->name);
		return -ENODEV;
	}
	
	if (!device_is_ready(cfg->reset.port)) {
		LOG_DBG("%s not ready", cfg->reset.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&cfg->irq, (GPIO_INPUT | GPIO_ACTIVE_LOW | GPIO_PULL_UP));
	if (ret) {
		return ret;
	}
	
	ret = gpio_pin_configure_dt(&cfg->wake, GPIO_OUTPUT_HIGH);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_HIGH);
	if (ret) {
		return ret;
	}

    bno08x_reset(dev);
	// bno08x_wait_for_int(dev);

    /*  Open SH2 interface (also registers non-sensor event handler.)
		Performs a soft reset of the sensor and runs the
		service once to get initial data.
	*/
	LOG_INF("sh2_open");
    err = sh2_open(&sh2_HAL, sh2_callback, NULL, dev);
    if (err != SH2_OK) {
        LOG_ERR("Cannot open SH2 dev: %d", err);
        return -ENODEV;
    }

    // Check connection partially by getting the product id's
    memset(&productIds, 0, sizeof(productIds));
	// LOG_ERR("sh2_getProdIds");
    err = sh2_getProdIds(&productIds);
    if (err != SH2_OK) {
		LOG_ERR("Cannot get device id for SH2 dev: %d", err);
        return -ENODEV;
    }

    sh2_setSensorCallback(sh2_sensorHandler, NULL, dev);

	enableReport(SH2_ROTATION_VECTOR, SAMPLE_INTERVAL_US, 0, dev);
	enableReport(SH2_ACCELEROMETER, SAMPLE_INTERVAL_US, 0, dev);
	enableReport(SH2_GYROSCOPE_CALIBRATED, SAMPLE_INTERVAL_US, 0, dev);
	enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, SAMPLE_INTERVAL_US, 0, dev);


	LOG_INF("BNO08X init done");
	return ret;
}

static const struct sensor_driver_api bno08x_driver_api = {
	.sample_fetch = bno08x_sample_fetch,
	.channel_get = bno08x_channel_get,
	.attr_set = bno08x_attr_set,
};


#define BNO08X_CONFIG_INT(inst)

/* Initializes a struct bno08x_config for an instance on a SPI bus. */
#define BNO08X_CONFIG_SPI(inst)				\
	.bus.spi = SPI_DT_SPEC_INST_GET(		\
		inst, BNO08X_SPI_OPERATION, 0),		\
	.bus_io = &bno08x_bus_io_spi,			\
	.irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios), \
	.wake = GPIO_DT_SPEC_INST_GET(inst, wake_gpios), \
	.reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios), \

/* Initializes a struct bno08x_config for an instance on an I2C bus. */
#define BNO08X_CONFIG_I2C(inst)				\
	.bus.i2c = I2C_DT_SPEC_INST_GET(inst),		\
	.bus_io = &bno08x_bus_io_i2c,				\
	.irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios), \

#define BNO08X_CREATE_INST(inst)					\
									\
	static struct bno08x_data bno08x_drv_##inst;			\
									\
	static const struct bno08x_config bno08x_config_##inst = {	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			    (BNO08X_CONFIG_SPI(inst)),			\
			    (BNO08X_CONFIG_I2C(inst)))			\
		BNO08X_CONFIG_INT(inst)					\
	};								\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			      bno08x_init,				\
			      NULL,					\
			      &bno08x_drv_##inst,			\
			      &bno08x_config_##inst,			\
			      POST_KERNEL,				\
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &bno08x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BNO08X_CREATE_INST);
