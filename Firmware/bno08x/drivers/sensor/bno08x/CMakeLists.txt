zephyr_library()

zephyr_library_sources(bno08x.c)
zephyr_library_sources(sh2/sh2.c)
zephyr_library_sources(sh2/shtp.c)
zephyr_library_sources(sh2/sh2_util.c)
zephyr_library_sources(sh2/sh2_SensorValue.c)
zephyr_library_sources(sh2/euler.c)
zephyr_library_sources_ifdef(CONFIG_BNO08X_BUS_I2C bno08x_i2c.c)
zephyr_library_sources_ifdef(CONFIG_BNO08X_BUS_SPI bno08x_spi.c)
