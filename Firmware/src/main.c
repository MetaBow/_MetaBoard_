/*
 * Main application with ADPCM audio compression
 * Only necessary changes added to original main.c
 */

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

// Include ADPCM codec instead of LZ4
#include "adpcm_codec.h"

// Include DRV2605 library
#include "drv2605.h"
#include "haptic_cmd.h"

// battery monitor includes
#include <zephyr/bluetooth/services/bas.h>
#include "battery_monitor.h"

#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

//for testing on DK make this 1 and for testing on PCB make it 0
#define TEST_DK_APP				0

// #define DEBUG_PRINT				0

#define LOG_MODULE_NAME metabow
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
/* Lower number = higher priority; negative = cooperative (avoid long cooperatives) */
#define BLE_THREAD_PRIORITY 1
#define IMU_THREAD_PRIORITY 2

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define DFU_STATUS_LED DK_LED2
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define BATTERY_SERVICE_UPDATE_INTERVAL_MS 60000

//Audio

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     500

/* Size of a block for N ms of audio data. This dictates our minimum latency */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	((BYTES_PER_SAMPLE * (90) * _number_of_channels))

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)   /* 180 bytes */
#define BLOCK_COUNT      32


// Quaternion, Acceleration, Gyroscope, Magnetometer
// IMU payload layout (floats):
// [0..3]   Quaternion (I, J, K, R)
// [4..6]   Linear Acceleration (m/s^2, gravity removed)  <-- unchanged indices
// [7..9]   Gyroscope (rad/s)
// [10..12] Magnetometer (uT)
// [13..15] RAW Acceleration (m/s^2, gravity included)   <-- new
#define IMU_DATA_SIZE (16)*sizeof(float)
#define IMU_DATA_FLAG_SIZE 1
#define BATTERY_DATA_SIZE sizeof(float)  // Battery SoC as float

// ADPCM compression defines
#define PCM_SAMPLES_PER_BLOCK (MAX_BLOCK_SIZE / BYTES_PER_SAMPLE)     /* 90 */
#define ADPCM_BLOCK_SIZE      ((PCM_SAMPLES_PER_BLOCK + 1) / 2)       /* 45 */
#define BLE_BLOCK_SIZE        (ADPCM_BLOCK_SIZE + IMU_DATA_SIZE + IMU_DATA_FLAG_SIZE + BATTERY_DATA_SIZE) /* 102 */


int bno08x_get_raw_accel(const struct device *dev, struct sensor_value out[3]);

/* Separate slabs: one for DMIC PCM blocks, one for BLE packets */
K_MEM_SLAB_DEFINE(pcm_mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);
/* 96 packets ≈ 9.8 KB; safe on nRF5340 app RAM */
K_MEM_SLAB_DEFINE(ble_mem_slab, BLE_BLOCK_SIZE, 96, 4);

// ADPCM encoder state (global for continuous encoding)
static adpcm_encoder_state_t adpcm_encoder_state;

static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

const struct device *const imu_dev = DEVICE_DT_GET(DT_NODELABEL(bno085));

#if !DT_NODE_EXISTS(DT_NODELABEL(bno085))
#error "bno08x not defined in device tree"
#endif

K_PIPE_DEFINE(imu_pipe, IMU_DATA_SIZE, 4);

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
     // 🔧 Ensure encoder/decoder state sync at the moment we start streaming:
    adpcm_encoder_init(&adpcm_encoder_state);
	
    // Start battery level updates when connected
    k_work_reschedule(&battery_ble_update_work, K_NO_WAIT);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != current_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	dk_set_led_off(CON_STATUS_LED);
	
	// Stop battery updates when disconnected
	k_work_cancel_delayable(&battery_ble_update_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	/* --- HAPTIC COMMANDS: try to handle and return if handled --- */
	char cmd[64];
	uint16_t n = MIN(len, sizeof(cmd)-1);
	memcpy(cmd, data, n);
	cmd[n] = 0;
	if (haptic_cmd_handle(cmd)) {
		return; /* handled (e.g., "V47", "BUZZ 200 180", "STOP", etc.) */
	}
	/* --- otherwise fall through to your existing FIFO forwarding --- */
	
	struct mem_slab_data_t *tx_data = k_malloc(sizeof(struct mem_slab_data_t));
	tx_data->len = len;
	tx_data->data = k_malloc(len);
	memcpy(tx_data->data, data, tx_data->len);
	k_fifo_put(&fifo_nus_tx_data, tx_data);
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

void trigger_dmic_data_ready(void)
{
	k_sem_give(&dmic_data_available);
}

void dmic_thread(void)
{
	int ret;
	void *mem_block;
	uint32_t block_size;
	int32_t timeout;
	
	// Initialize ADPCM encoder at thread start
	adpcm_encoder_init(&adpcm_encoder_state);
	LOG_INF("ADPCM encoder initialized for audio compression");

	
	/* Acquire microphone audio */
	for (;;) {
		ret = dmic_read(dmic_dev, 0, &mem_block, &block_size, READ_TIMEOUT);
		if (ret < 0) {
			LOG_WRN("Audio buf read err: %d", ret);
			continue;
		}

		LOG_DBG("Audio block received: %d bytes (%d samples)", 
			block_size, block_size / BYTES_PER_SAMPLE);

		// Signal that data is ready
		trigger_dmic_data_ready();
		
		// Allocate BLE packet buffer (compressed audio + IMU + flag + battery)
		struct mem_slab_data_t *buf = k_malloc(sizeof(struct mem_slab_data_t));
		if(buf != NULL){
            ret = k_mem_slab_alloc(&ble_mem_slab, &buf->data, K_NO_WAIT);
			if (ret != 0) {
				LOG_ERR("Failed to allocate mem_slab: %d", ret);
                /* Drop this audio frame; keep DMIC flowing */
               /* Optional: count drops with a static counter and log occasionally */
				k_free(buf);
                goto free_pcm_and_continue;
			} else {
				uint8_t *ble_buffer = (uint8_t *)buf->data;
				
				// Compress audio using ADPCM (first part of packet)
				size_t compressed_size = adpcm_encode(
					(const int16_t *)mem_block,
					block_size / BYTES_PER_SAMPLE,  // Number of PCM samples
					ble_buffer,  // Output at start of BLE buffer
					&adpcm_encoder_state
				);
				
				LOG_DBG("ADPCM compression: %d PCM bytes -> %d ADPCM bytes", 
					block_size, compressed_size);
				
				buf->len = BLE_BLOCK_SIZE;  // Full packet size
				k_fifo_put(&fifo_nus_rx_data, buf);
			}
		}

		// Free the PCM memory block back to the DMIC slab
        free_pcm_and_continue:
        k_mem_slab_free(&pcm_mem_slab, &mem_block);
	}
}

void ble_write_thread(void)
{
    /* Don't go any further until BLE is initialized */
    k_sem_take(&ble_init_ok, K_FOREVER);
    int rc;
    
    LOG_INF("BLE write thread started with ADPCM compression");
    LOG_INF("Packet structure: [%d bytes ADPCM][%d bytes IMU][1 byte flag][%d bytes battery]",
            ADPCM_BLOCK_SIZE, IMU_DATA_SIZE, BATTERY_DATA_SIZE);
    LOG_INF("Total packet size: %d bytes (vs %d uncompressed)", 
            BLE_BLOCK_SIZE, MAX_BLOCK_SIZE + IMU_DATA_SIZE + IMU_DATA_FLAG_SIZE + BATTERY_DATA_SIZE);
    
    for (;;) {
        struct mem_slab_data_t *buf = k_fifo_get(&fifo_nus_rx_data, K_FOREVER);
        if(buf != NULL){
            void *buffer = buf->data;
            uint32_t size = BLE_BLOCK_SIZE;

            // Get IMU data
            size_t bytes_read;
            rc = k_pipe_get(&imu_pipe, (uint8_t*)buffer+ADPCM_BLOCK_SIZE, IMU_DATA_SIZE, &bytes_read,
                 IMU_DATA_SIZE, K_NO_WAIT);
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
            *((uint8_t*)buffer + ADPCM_BLOCK_SIZE + IMU_DATA_SIZE) = imu_data_flag;
            
            // Add battery SoC data
            float battery_soc = (float)battery_get_soc();  // Get current battery percentage
            memcpy((uint8_t*)buffer + ADPCM_BLOCK_SIZE + IMU_DATA_SIZE + IMU_DATA_FLAG_SIZE, 
                   &battery_soc, BATTERY_DATA_SIZE);
#if defined(DEBUG_PRINT)
            LOG_INF("Sending BLE data with Battery SoC: %.1f%%", battery_soc);
#endif
            size_t max_packet_size = 0;
            if (current_conn) {
                max_packet_size = bt_nus_get_mtu(current_conn);
            }
#if defined(DEBUG_PRINT)
            LOG_INF("BLE audio data buffer size: %d, MTU size: %d", size, (int)max_packet_size);
#endif
            // CRITICAL FIX: Only send if connected
            if (current_conn) {
                if (size > max_packet_size){
                    for (uint32_t sendIndex = 0; sendIndex < size; sendIndex += max_packet_size) {
                        uint32_t chunkLength = sendIndex + max_packet_size < size
                                ? max_packet_size
                                : (size - sendIndex);
                        if (bt_nus_send(current_conn, (uint8_t*)buffer + sendIndex, chunkLength)) {
                            // LOG_WRN("Failed to send audio data over BLE connection");
                        }
                    }
                }else{
                    if (bt_nus_send(current_conn, (uint8_t*)buffer, size)) {
                        // LOG_WRN("Failed to send audio data over BLE connection");
                    }
                }
            }
            // Free BLE packet buffer back to BLE slab
            k_mem_slab_free(&ble_mem_slab, &buffer);
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
    struct sensor_value accel[3];      /* linear accel (existing behavior) */
    struct sensor_value accel_raw[3];  /* new: raw accel */
	struct sensor_value gyro[3];
	struct sensor_value mag[3];
	float imu_data[IMU_DATA_SIZE/sizeof(float)];
	size_t bytes_written;
	int rc;
	
	LOG_INF("IMU thread started");
	
	for (;;) {
		
		sensor_sample_fetch(imu_dev);

		rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ROTATION_VEC_IJKR, quat);
		if (rc < 0){LOG_ERR("could not get ROTATION_VEC data: %d", rc);continue;}
        /* Unchanged: ACCEL_XYZ gives LINEAR accel (gravity removed) */
        rc = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (rc < 0){LOG_ERR("could not get ACCEL_XYZ data: %d", rc);continue;}
		/* RAW (gravity-included) accel via driver helper */
	    rc = bno08x_get_raw_accel(imu_dev, accel_raw);
	    if (rc < 0){LOG_ERR("could not get RAW accel: %d", rc); continue;}

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

        /* Append RAW acceleration */
        imu_data[13] = (float)sensor_value_to_double(&accel_raw[0]);
        imu_data[14] = (float)sensor_value_to_double(&accel_raw[1]);
        imu_data[15] = (float)sensor_value_to_double(&accel_raw[2]);


		rc = k_pipe_put(&imu_pipe, imu_data, IMU_DATA_SIZE, &bytes_written, IMU_DATA_SIZE, K_FOREVER);
		if (rc < 0) {
            LOG_ERR("Failed to put IMU data into pipe: %d", rc);
        } else if (bytes_written < IMU_DATA_SIZE) {
            LOG_ERR("Only %d bytes written to IMU pipe", bytes_written);
        }
#if defined(DEBUG_PRINT)
		LOG_INF("Rotation: I: %f, J: %f, K: %f, R: %f", sensor_value_to_double(&quat[0]), sensor_value_to_double(&quat[1]), sensor_value_to_double(&quat[2]), sensor_value_to_double(&quat[3]));
		LOG_INF("Acceleration: X: %f, Y: %f, Z: %f", sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]), sensor_value_to_double(&accel[2]));
		LOG_INF("Raw Accel: X: %f, Y: %f, Z: %f", sensor_value_to_double(&accel_raw[0]), sensor_value_to_double(&accel_raw[1]), sensor_value_to_double(&accel_raw[2]));
		LOG_INF("Gyroscope: X: %f, Y: %f, Z: %f", sensor_value_to_double(&gyro[0]), sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
		LOG_INF("Magnetometer: X: %f, Y: %f, Z: %f", sensor_value_to_double(&mag[0]), sensor_value_to_double(&mag[1]), sensor_value_to_double(&mag[2]));
#endif
		// bt_nus_send(current_conn, (uint8_t*) quat, sizeof(quat));
		k_sleep(K_USEC(200));
	}
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

// --- add this block above main() ---
static void haptic_init_work_handler(struct k_work *work)
{
    int err = drv2605_init_from_dt(NULL);
    if (err == 0) {
        /* Do not auto-play any effect here to avoid load/IRQ contention */
        LOG_DBG("DRV2605 init successful (idle)");
    } else {
        LOG_ERR("DRV2605 init failed (delayed): %d", err);
    }
}
K_WORK_DELAYABLE_DEFINE(haptic_init_work, haptic_init_work_handler);

/* Dedicated low-priority workqueue for haptic operations */
static struct k_work_q haptic_wq;
K_THREAD_STACK_DEFINE(haptic_wq_stack, 1024);
// --- end block ---

int main(void)
{
	int blink_status = 0;
	int err = 0;
	int ret;

	LOG_INF("Starting Metabow with ADPCM audio compression");
	LOG_INF("Compression ratio: 4:1 (16-bit PCM to 4-bit ADPCM)");
	LOG_INF("Total packet size: %d bytes", BLE_BLOCK_SIZE);
	
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
#endif


    struct pcm_stream_cfg stream = {
        .pcm_width = SAMPLE_BIT_WIDTH,
        .mem_slab  = &pcm_mem_slab,  /* DMIC must use 180-byte blocks */
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

	err = dmic_configure(dmic_dev, &cfg);
	if (err < 0) {
		LOG_ERR("Failed to configure the driver: %d", err);
		return err;
	}
	
	nrf_pdm_gain_set(NRF_PDM0, NRF_PDM_GAIN_MAXIMUM, NRF_PDM_GAIN_MAXIMUM);

	LOG_INF("DMIC thread started with ADPCM compression");

    
	// Initialize battery monitoring
	err = battery_monitor_init();
	if (err) {
		LOG_ERR("Battery monitor init failed (err: %d)", err);
		return 0;
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}

	/* Start dedicated low-priority haptic workqueue (higher number = lower priority) */
	k_work_queue_start(&haptic_wq, haptic_wq_stack,
		K_THREAD_STACK_SIZEOF(haptic_wq_stack), 10, NULL);
	k_thread_name_set(&haptic_wq.thread, "haptic_wq");

	// Delay haptic init to avoid interfering with IMU/BLE bring-up
	k_work_schedule_for_queue(&haptic_wq, &haptic_init_work, K_SECONDS(2));
		

	err = settings_subsys_init();
	if (err) {
		LOG_ERR("settings_subsys_init: %d", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	LOG_INF("Configuration complete. Starting operation with ADPCM compression.");
	
	// Initialize battery work handler
	k_work_init_delayable(&battery_ble_update_work, battery_ble_update_handler);

	err = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (err < 0) {
		LOG_ERR("START trigger failed: %d", err);
		return err;
	}
	


	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

#if (TEST_DK_APP == 0)
K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, BLE_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(imu_fetch_thread_id, 4096, imu_fetch_thread, NULL, NULL,
		NULL, IMU_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(dmic_thread_id, 8192, dmic_thread, NULL, NULL,
		NULL, 0, 0, 0);
#endif