#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/audio/dmic.h>

#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <zephyr/drivers/gpio.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

// needed to set gain
#include <nrfx_pdm.h>

#include "lz4.h"

// battery monitor includes
#include <zephyr/bluetooth/services/bas.h>
#include "battery_monitor.h"

#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

//for testing on DK make this 1 and for testing on PCB make it 0
#define TEST_DK_APP				0

#define LOG_MODULE_NAME metabow
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define BLE_THREAD_PRIORITY -1
#define IMU_THREAD_PRIORITY 0

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define DFU_STATUS_LED DK_LED2
#
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

// #define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
// #define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
// #define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

//Audio

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     500

/* Size of a block for N ms of audio data. This dictates our minimum latency */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	((BYTES_PER_SAMPLE * (90) * _number_of_channels))
	// ((BYTES_PER_SAMPLE * (_sample_rate /160) * _number_of_channels))

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      32

// Quaternion, Acceleration, Gyroscope, Magnetometer
// #define IMU_DATA_SIZE (4+3+3+3)*sizeof(float)
#define IMU_DATA_SIZE (13)*sizeof(float)
#define IMU_DATA_FLAG_SIZE 1
#define BATTERY_DATA_SIZE sizeof(float)  // Battery SoC as float
#define BLE_BLOCK_SIZE MAX_BLOCK_SIZE+IMU_DATA_SIZE+IMU_DATA_FLAG_SIZE+BATTERY_DATA_SIZE

K_MEM_SLAB_DEFINE(mem_slab, BLE_BLOCK_SIZE, BLOCK_COUNT, 4);

static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

const struct device *const imu_dev = DEVICE_DT_GET(DT_NODELABEL(bno085));

#if !DT_NODE_EXISTS(DT_NODELABEL(bno085))
#error "bno08x not defined in device tree"
#endif

K_PIPE_DEFINE(imu_pipe, IMU_DATA_SIZE, 4);

// #define IMU_CLK_NODE DT_ALIAS(imu_clk_sel_1)
// #define IMU_CLK_NODE DT_NODELABEL(imu_clk_sel_1)
// static const struct gpio_dt_spec imu_clk_sel = GPIO_DT_SPEC_GET(IMU_CLK_NODE, gpios);


// #if !DT_NODE_EXISTS(IMU_CLK_NODE)
// #error "Overlay for clk_sel node not properly defined."
// #endif



// BLE

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_SEM_DEFINE(imu_init_ok, 0, 1);
static K_SEM_DEFINE(dmic_data_available, 0, BLOCK_COUNT);

// Battery BLE update work
static struct k_work_delayable battery_ble_update_work;


static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

struct mem_slab_data_t {
	void *fifo_reserved;
	void *data;
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_nus_tx_data);
static K_FIFO_DEFINE(fifo_nus_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

// static const char *phy2str(uint8_t phy)
// {
// 	switch (phy) {
// 	case 0: return "No packets";
// 	case BT_GAP_LE_PHY_1M: return "LE 1M";
// 	case BT_GAP_LE_PHY_2M: return "LE 2M";
// 	case BT_GAP_LE_PHY_CODED: return "LE Coded";
// 	default: return "Unknown";
// 	}
// }

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);

    current_conn = bt_conn_ref(conn);

    dk_set_led_on(CON_STATUS_LED);
    
    // Start battery level updates when connected
    k_work_reschedule(&battery_ble_update_work, K_NO_WAIT);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason %u)", addr, reason);

    if (auth_conn) {
        bt_conn_unref(auth_conn);
        auth_conn = NULL;
    }

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        dk_set_led_off(CON_STATUS_LED);
        
        // Stop battery updates when disconnected
        k_work_cancel_delayable(&battery_ble_update_work);
    }
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	LOG_INF("Connection parameters update request received.\n");
	LOG_INF("Minimum interval: %d, Maximum interval: %d\n",
	       param->interval_min, param->interval_max);
	LOG_INF("Latency: %d, Timeout: %d\n", param->latency, param->timeout);

	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	LOG_INF("Connection parameters updated.\n"
	       " interval: %d, latency: %d, timeout: %d\n",
	       interval, latency, timeout);

}

// static void le_phy_updated(struct bt_conn *conn,
// 			   struct bt_conn_le_phy_info *param)
// {
// 	LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s\n",
// 	       phy2str(param->tx_phy), phy2str(param->rx_phy));

// }

// static void le_data_length_updated(struct bt_conn *conn,
// 				   struct bt_conn_le_data_len_info *info)
// {
// 	LOG_INF("LE data len updated: TX (len: %d time: %d)"
// 	       " RX (len: %d time: %d)\n", info->tx_max_len,
// 	       info->tx_max_time, info->rx_max_len, info->rx_max_time);

// }

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    LOG_INF("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
        .att_mtu_updated = mtu_updated
};

// static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
// {
// 	if (!err) {
// 		LOG_INF("MTU exchange done");
// 	} else {
// 		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
// 	}
// }

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	// .le_phy_updated = le_phy_updated,
	// .le_data_len_updated = le_data_length_updated,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

static void configure_gpio(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

// Example: Get battery status periodically
void get_battery_status(void)
{
    uint8_t soc = battery_get_soc();
    float voltage = battery_get_voltage();
    uint16_t raw_adc = battery_get_raw_adc();
    
    LOG_INF("Battery Status - SoC: %d%%, Voltage: %.2fV, ADC: %d", 
            soc, voltage, raw_adc);
    
    // You can send this over BLE or use it in your application
}



static void battery_ble_update_handler(struct k_work *work)
{
    uint8_t battery_level = battery_get_soc();
    float battery_voltage = battery_get_voltage();
    
    // Update BLE Battery Service
    int err = bt_bas_set_battery_level(battery_level);
    if (err) {
        LOG_WRN("Failed to update battery level: %d", err);
    } else {
        LOG_INF("BLE Battery Service updated: %d%% (%.2fV)", battery_level, battery_voltage);
    }
    
    // Reschedule for next update
    k_work_reschedule(&battery_ble_update_work, K_MSEC(BATTERY_SERVICE_UPDATE_INTERVAL_MS));
}


int main(void)
{
	int blink_status = 0;
	int err = 0;
	int ret;

	
#if (TEST_DK_APP == 0)
	if (!device_is_ready(dmic_dev)) {
		LOG_ERR("%s is not ready", dmic_dev->name);
		return 0;
	}

	if (!device_is_ready(imu_dev)) {
		LOG_ERR("Device %s is not ready\n", imu_dev->name);
		return 0;
	}

	k_sem_give(&imu_init_ok);

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};

	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1200000,
			.max_pdm_clk_freq = 3200000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	LOG_INF("PCM output rate: %u, channels: %u",
		cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure the driver: %d", ret);
		return ret;
	}
	
	nrf_pdm_gain_set(NRF_PDM0, NRF_PDM_GAIN_MAXIMUM, NRF_PDM_GAIN_MAXIMUM);
#endif
	configure_gpio();

	// err = uart_init();
	// if (err) {
	// 	error();
	// }

	err = bt_enable(NULL);
	if (err) {
		error();
	}
	// /* ---------- expose mcumgr SMP DFU service -------------- */
	smp_bt_register();        /* init Secure DFU OTA */


	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize NUS service (err: %d)", err);
		return 0;
	}

	k_sleep(K_MSEC(500));
	// Initialize battery monitoring
	err = battery_monitor_init();
	if (err) {
		LOG_ERR("Battery monitor init failed: %d", err);
		// Continue anyway, battery monitoring is not critical
	}

	    // Initialize battery BLE update work
    k_work_init_delayable(&battery_ble_update_work, battery_ble_update_handler);

    // Get initial battery reading and set it
    k_sleep(K_MSEC(100)); // Small delay to ensure battery monitor has a reading
    uint8_t initial_battery = battery_get_soc();
    err = bt_bas_set_battery_level(initial_battery);
    if (err) {
        LOG_WRN("Failed to set initial battery level: %d", err);
    } else {
        LOG_INF("Initial battery level set to %d%%", initial_battery);
    }

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	bt_gatt_cb_register(&gatt_callbacks);

	//===TESTING=======================================
	// if (!gpio_is_ready_dt(&imu_clk_sel)) {
	// 	return 0;
	// }

	// ret = gpio_pin_configure_dt(&imu_clk_sel, GPIO_OUTPUT_INACTIVE);
	// if (ret < 0) {
	// 	return 0;
	// }

	//==================================================
#if (TEST_DK_APP == 0)
	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("START trigger failed: %d", ret);
		return ret;
	}
#endif
	

	
	for (;;) {

		// battery status
		// get_battery_status();
		
#if (TEST_DK_APP == 0)
		// dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		// k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
#else
		dk_set_led(DFU_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
#endif

		// ---------------------------------------------
#if (TEST_DK_APP == 0)

		void *buffer;
		uint32_t size;
#if defined(DEBUG_PRINT)
		LOG_INF("mem_slabs in use before next dmic read: %d", k_mem_slab_num_used_get(&mem_slab));
#endif
		ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
		if (ret < 0) {
			LOG_ERR("dmic read failed: %d", ret);
			return ret;
		}
		
		struct mem_slab_data_t *tx = k_malloc(sizeof(*tx));
		if (tx == NULL) {
			LOG_ERR("unable to allocate memory for mem_slab_data_t %d", ret);
			return ret;
		}
		tx->len = size;
		tx->data = buffer;
		k_fifo_put(&fifo_nus_rx_data, tx);
		// LOG_INF("dmic buffer size: %d", size);
#endif

	}
#if (TEST_DK_APP == 0)

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		LOG_ERR("STOP trigger failed: %d", ret);
		return ret;
	}
#endif

}

void ble_write_thread(void)
{
    /* Don't go any further until BLE is initialized */
    k_sem_take(&ble_init_ok, K_FOREVER);
    int rc;
    
    for (;;) {
        struct mem_slab_data_t *buf = k_fifo_get(&fifo_nus_rx_data, K_FOREVER);
        if(buf != NULL){
            void *buffer = buf->data;
            uint32_t size = BLE_BLOCK_SIZE;

            // Get IMU data
            size_t bytes_read;
            rc = k_pipe_get(&imu_pipe, (uint8_t*)buffer+MAX_BLOCK_SIZE, IMU_DATA_SIZE, &bytes_read,
                    IMU_DATA_SIZE, K_USEC(50));
            uint8_t imu_data_flag = 0;
            if((rc < 0) && (bytes_read == 0)){
                imu_data_flag = 0;
            }else if ((rc < 0) || (bytes_read < IMU_DATA_SIZE)) {
                LOG_ERR("Failed to get all IMU data from pipe, read: %d", bytes_read);
                imu_data_flag = 0;
            }else{
                imu_data_flag = 1;
            }
            
            // Set IMU data flag
            *((uint8_t*)buffer + MAX_BLOCK_SIZE + IMU_DATA_SIZE) = imu_data_flag;
            
            // Add battery SoC data
            float battery_soc = (float)battery_get_soc();  // Get current battery percentage
            memcpy((uint8_t*)buffer + MAX_BLOCK_SIZE + IMU_DATA_SIZE + IMU_DATA_FLAG_SIZE, 
                   &battery_soc, BATTERY_DATA_SIZE);
            
            LOG_INF("Sending BLE data with Battery SoC: %.1f%%", battery_soc);

            const size_t max_packet_size = bt_nus_get_mtu(current_conn);
            LOG_INF("BLE audio data buffer size: %d, MTU size: %d", size, max_packet_size);
            
            if (size > max_packet_size){
                for (uint32_t sendIndex = 0; sendIndex < size; sendIndex += max_packet_size) {
                    uint32_t chunkLength = sendIndex + max_packet_size < size
                            ? max_packet_size
                            : (size - sendIndex);
                    if (bt_nus_send(current_conn, (uint8_t*) buffer + sendIndex, chunkLength)) {
                        // LOG_WRN("Failed to send audio data over BLE connection");
                    }
                }
            }else{
                if (bt_nus_send(current_conn, (uint8_t*) buffer, size)) {
                    // LOG_WRN("Failed to send audio data over BLE connection");
                }
            }
            k_mem_slab_free(&mem_slab, &buffer);
            k_free(buf);
        }
    }
}

void imu_fetch_thread(void)
{
	k_sem_take(&imu_init_ok, K_FOREVER);
	// todo add out of tree sensor_channel include to define custom channels
	#define SENSOR_CHAN_ROTATION_VEC_IJKR 61
	struct sensor_value quat[4];
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value mag[3];
	float imu_data[IMU_DATA_SIZE/sizeof(float)];
	size_t bytes_written;
	int rc;
	for (;;) {
		
		sensor_sample_fetch(imu_dev);

		rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ROTATION_VEC_IJKR, quat);
		if (rc < 0){LOG_ERR("could not get ROTATION_VEC data: %d", rc);continue;}
		rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (rc < 0){LOG_ERR("could not get ACCEL_XYZ data: %d", rc);continue;}
		rc = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		if (rc < 0){LOG_ERR("could not get GYRO_XYZ data: %d", rc);continue;}
		rc = sensor_channel_get(imu_dev, SENSOR_CHAN_MAGN_XYZ, mag);
		if (rc < 0){LOG_ERR("could not get MAGN_XYZ data: %d", rc);continue;}

		// TBD should become a struct
		imu_data[0] = (float)sensor_value_to_double(&quat[0]);
		imu_data[1] = (float)sensor_value_to_double(&quat[1]);
		imu_data[2] = (float)sensor_value_to_double(&quat[2]);
		imu_data[3] = (float)sensor_value_to_double(&quat[3]);

		imu_data[4] = (float)sensor_value_to_double(&accel[0]);
		imu_data[5] = (float)sensor_value_to_double(&accel[1]);
		imu_data[6] = (float)sensor_value_to_double(&accel[2]);

		imu_data[7] = (float)sensor_value_to_double(&gyro[0]);
		imu_data[8] = (float)sensor_value_to_double(&gyro[1]);
		imu_data[9] = (float)sensor_value_to_double(&gyro[2]);

		imu_data[10] = (float)sensor_value_to_double(&mag[0]);
		imu_data[11] = (float)sensor_value_to_double(&mag[1]);
		imu_data[12] = (float)sensor_value_to_double(&mag[2]);

		rc = k_pipe_put(&imu_pipe, imu_data, IMU_DATA_SIZE, &bytes_written, IMU_DATA_SIZE, K_FOREVER);
		
		if (rc < 0) {
            LOG_ERR("Failed to put IMU data into pipe: %d", rc);
        } else if (bytes_written < IMU_DATA_SIZE) {
            LOG_ERR("Only %d bytes written to IMU pipe", bytes_written);
        }
#if defined(DEBUG_PRINT)
		LOG_INF("Rotation: I: %f, J: %f, K: %f, R: %f", sensor_value_to_double(&quat[0]), sensor_value_to_double(&quat[1]), sensor_value_to_double(&quat[2]), sensor_value_to_double(&quat[3]));
		LOG_INF("Acceleration: X: %f, Y: %f, Z: %f", sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]), sensor_value_to_double(&accel[2]));
		LOG_INF("Gyroscope: X: %f, Y: %f, Z: %f", sensor_value_to_double(&gyro[0]), sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
		LOG_INF("Magnetometer: X: %f, Y: %f, Z: %f", sensor_value_to_double(&mag[0]), sensor_value_to_double(&mag[1]), sensor_value_to_double(&mag[2]));
#endif
		// bt_nus_send(current_conn, (uint8_t*) quat, sizeof(quat));
		k_sleep(K_USEC(200));
	}
}
#if (TEST_DK_APP == 0)
K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, BLE_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(imu_fetch_thread_id, 4096, imu_fetch_thread, NULL, NULL,
		NULL, IMU_THREAD_PRIORITY, 0, 0);
#endif