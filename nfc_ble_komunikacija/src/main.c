#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <nfc_t2t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>
#include <dk_buttons_and_leds.h>
#include <stdio.h>
#include <stdbool.h>
#include <nfc_t4t_lib.h>
#include <nfc/t4t/ndef_file.h>
#include "ndef_file_m.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/latency.h>
#include <bluetooth/services/latency_client.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

#define SLEEP_TIME          5
#define MAX_REC_COUNT       3
#define NDEF_MSG_BUF_SIZE   1024
#define NFC_FIELD_LED       DK_LED1
#define NFC_FIELD_LED_TWO       DK_LED2
#define MAX_SECONDS_ELAPSED 1000

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define INTERVAL_MIN      0x6    /* 6 units,  7.5 ms */
#define INTERVAL_MIN_US  7500    /* 7.5 ms */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_GATT_VAL)),
};

static uint8_t ndef_msg_buf[CONFIG_NDEF_FILE_SIZE]; /**< Buffer for NDEF file. */
static uint8_t ndef_msg_bufTEST[CONFIG_NDEF_FILE_SIZE]; /**< Buffer for NDEF file. */

enum {
    FLASH_WRITE_FINISHED,
    FLASH_BUF_PREP_STARTED,
    FLASH_BUF_PREP_FINISHED,
    FLASH_WRITE_STARTED,
};
static atomic_t op_flags;
static uint8_t flash_buf[CONFIG_NDEF_FILE_SIZE]; /**< Buffer for flash update. */
static uint8_t flash_buf_len; /**< Length of the flash buffer. */

static void flash_buffer_prepare(size_t data_length)
{   
    if (atomic_cas(&op_flags, FLASH_WRITE_FINISHED,
                   FLASH_BUF_PREP_STARTED)) {
        flash_buf_len = data_length + NFC_NDEF_FILE_NLEN_FIELD_SIZE;
        memcpy(flash_buf, ndef_msg_bufTEST, sizeof(flash_buf));

        atomic_set(&op_flags, FLASH_BUF_PREP_FINISHED);
    } else {
        printk("Flash update pending. Discarding new data...\n");
    }
}

/* Buffer used to hold an NFC NDEF message. */
static uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];

/* Dynamic payload buffer */
static uint8_t dynamic_payload[NDEF_MSG_BUF_SIZE];

static const uint8_t en_code[] = {'e', 'n'};
static int seconds_elapsed = 0;
static int seconds_list[MAX_SECONDS_ELAPSED];
static int seconds_count = 0;
char testString[NDEF_MSG_BUF_SIZE] = "payload:";

static ssize_t read_nfc_payload(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, dynamic_payload, strlen(dynamic_payload));
}

/* GATT service declaration */
BT_GATT_SERVICE_DEFINE(nfc_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0x1800)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x2A00),
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_nfc_payload, NULL, NULL),
);

/**
 * @brief Function for encoding the NDEF text message.
 */
static int encode_msg(uint8_t *buffer, uint32_t *len)
{
    snprintf(dynamic_payload, sizeof(dynamic_payload), "%s", testString);

    NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_en_text_rec, UTF_8, en_code, sizeof(en_code), dynamic_payload, strlen(dynamic_payload));
    NFC_NDEF_MSG_DEF(nfc_text_msg, MAX_REC_COUNT);

    int err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_text_msg), &NFC_NDEF_TEXT_RECORD_DESC(nfc_en_text_rec));
    if (err < 0) {
        return err;
    }

    err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_text_msg), buffer, len);
    return err;
}

static void nfc_callback(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t data_length)
{
    ARG_UNUSED(context);
    ARG_UNUSED(data);
    ARG_UNUSED(data_length);

    switch (event) {
    case NFC_T2T_EVENT_FIELD_ON:
        dk_set_led_on(NFC_FIELD_LED);
        break;

    case NFC_T2T_EVENT_FIELD_OFF:
        dk_set_led_off(NFC_FIELD_LED);
        break;

    default:
        break;
    }
}

static void ble_send_work_handler(struct k_work *work)
{
    static uint32_t len = sizeof(ndef_msg_buf);
    char temp[20];

    if (seconds_elapsed < MAX_SECONDS_ELAPSED) {
        seconds_elapsed++;
        seconds_list[seconds_count++] = seconds_elapsed;
    }

    snprintf(temp, sizeof(temp), "%d;", seconds_elapsed);
    strcat(testString, temp);
    len = NDEF_MSG_BUF_SIZE;
    nfc_t2t_emulation_stop();
    if (encode_msg(ndef_msg_buf, &len) < 0) {
        dk_set_led_on(NFC_FIELD_LED);
        printk("Cannot encode message!\n");
    } else {
        int ret = nfc_t2t_payload_set(ndef_msg_buf, len);
        if (ret < 0) {
            dk_set_led_on(NFC_FIELD_LED_TWO);
            printk("Cannot set payload!\n");
        }
    }

    // Prepare the flash buffer with the current message
    flash_buffer_prepare(len);

    if (ndef_file_update(flash_buf, flash_buf_len) < 0) {
        printk("Cannot update NDEF file\n");
        return;
    }

    if (ndef_file_load(ndef_msg_bufTEST, sizeof(ndef_msg_bufTEST)) < 0) {
        printk("Cannot load NDEF file!\n");
        return;
    }
    nfc_t2t_emulation_start();
}

K_WORK_DEFINE(ble_send_work, ble_send_work_handler);

void timer_handler(struct k_timer *dummy)
{
    k_work_submit(&ble_send_work);
}

K_TIMER_DEFINE(ble_send_timer, timer_handler, NULL);

void nfc_callback_with_wake(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t data_length)
{
    
    nfc_callback(context, event, data, data_length);

    k_timer_stop(&ble_send_timer);  // Stop the timer to wake up the system
    k_timer_start(&ble_send_timer, K_SECONDS(SLEEP_TIME), K_SECONDS(SLEEP_TIME));  // Restart the timer
}

void main(void)
{
    uint32_t len = sizeof(ndef_msg_buf);
    struct k_sem my_sem;

    printk("Starting NFC Text Record example\n");
    /* Configure LED-pins as outputs */
    if (dk_leds_init() < 0) {
        printk("Cannot init LEDs!\n");
        goto fail;
    }

    /* setup NDEF */
    if (ndef_file_setup() < 0) {
        printk("Cannot setup NDEF file!\n");
        goto fail;
    }

    /* Load NDEF message from the flash file. */
    if (ndef_file_load(ndef_msg_bufTEST, sizeof(ndef_msg_bufTEST)) < 0) {
        printk("Cannot load NDEF file!\n");
        goto fail;
    }

    /* Set up NFC */
    if (nfc_t2t_setup(nfc_callback_with_wake, NULL) < 0) {
        printk("Cannot setup NFC T2T library!\n");
        goto fail;
    }

    /* Encode message */
    if (encode_msg(ndef_msg_buf, &len) < 0) {
        printk("Cannot encode message!\n");
        goto fail;
    }

    /* Set created message as the NFC payload */
    if (nfc_t2t_payload_set(ndef_msg_buf, len) < 0) {
        printk("Cannot set payload!\n");
        goto fail;
    }

    /* Start sensing NFC field */
    if (nfc_t2t_emulation_start() < 0) {
        printk("Cannot start emulation!\n");
        goto fail;
    }
    printk("NFC configuration done\n");

    int errBle;
    /* Initialize the Bluetooth Subsystem */
    errBle = bt_enable(NULL);
    if (errBle) {
        return;
    }

    /* Start advertising */
    errBle = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (errBle) {
        return;
    }

    /* Start the timer for periodic BLE data sending */
    k_timer_start(&ble_send_timer, K_SECONDS(SLEEP_TIME), K_SECONDS(SLEEP_TIME));

    while (1) {
        // System goes to sleep until next event (timer or NFC)
        nfc_t2t_emulation_stop();
        static uint32_t len = NDEF_MSG_BUF_SIZE;
        if (encode_msg(ndef_msg_buf, &len) < 0) {
            printk("Cannot encode message!\n");
        } else {
            //len = sizeof(ndef_msg_buf);
            int ret = nfc_t2t_payload_set(ndef_msg_buf, len);
            if (ret < 0) {
                printk("Cannot set payload!\n");
            }
        }
        if (ndef_file_load(ndef_msg_bufTEST, sizeof(ndef_msg_bufTEST)) < 0) {
		printk("Cannot load NDEF file!\n");
	    }
        //ndef_msg_buf = ndef_msg_bufTEST; //+ ndef_msg_buf;
        if (ndef_file_update(ndef_msg_buf, sizeof(ndef_msg_buf)) < 0 ) {
        printk("Cannot update\n");
        }
        if (ndef_file_load(ndef_msg_bufTEST, sizeof(ndef_msg_bufTEST)) < 0) {
		printk("Cannot load NDEF file!\n");
	    }
            nfc_t2t_emulation_start();
        k_sleep(K_FOREVER);
    }

fail:
#if CONFIG_REBOOT
    sys_reboot(SYS_REBOOT_COLD);
#endif
    return;
}
