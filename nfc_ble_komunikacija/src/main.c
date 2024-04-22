#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <nfc_t2t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>
#include <dk_buttons_and_leds.h>
#include <stdio.h>

#include <stdbool.h>
#include <nfc_t4t_lib.h>

#include <nfc/t4t/ndef_file.h>

#include "ndef_file_m.h"


#define MAX_REC_COUNT       3
#define NDEF_MSG_BUF_SIZE   1024
#define NFC_FIELD_LED       DK_LED1
#define MAX_SECONDS_ELAPSED 1000

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
char testString[NDEF_MSG_BUF_SIZE]="payload:";


/**
 * @brief Function for encoding the NDEF text message.
 */
static int encode_msg(uint8_t *buffer, uint32_t *len)
{
    int strlenSize = strlen(dynamic_payload);
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
        strcpy(testString ,"payload:");
        dk_set_led_off(NFC_FIELD_LED);
        break;

    default:
        break;
    }
}

int main(void)
{
    uint32_t len = sizeof(ndef_msg_buf);
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
    /*
    if (ndef_file_update(ndef_msg_bufTEST, sizeof(ndef_msg_bufTEST)) < 0 ) {
        printk("Cannot update\n");
        goto fail;
    }
    */
	if (ndef_file_load(ndef_msg_bufTEST, sizeof(ndef_msg_bufTEST)) < 0) {
		printk("Cannot load NDEF file!\n");
		goto fail;
	}

    /* Set up NFC */
    if (nfc_t2t_setup(nfc_callback, NULL) < 0) {
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

    while (1) {
        k_sleep(K_SECONDS(1));  // Sleep for 1 second

        if (seconds_elapsed < MAX_SECONDS_ELAPSED) {
            seconds_elapsed++;
            seconds_list[seconds_count++] = seconds_elapsed;
        }
        // Encode and update NFC payload
        nfc_t2t_emulation_stop();  // Stop emulation to adjust the new payload
        char temp[20];
        snprintf(temp, sizeof(temp), "%d;", seconds_elapsed);
        strcat(testString,temp);
        len = NDEF_MSG_BUF_SIZE;
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
		goto fail;
	    }
        ndef_msg_buf = ndef_msg_bufTEST + ndef_msg_buf;
        if (ndef_file_update(ndef_msg_buf, sizeof(ndef_msg_buf)) < 0 ) {
        printk("Cannot update\n");
        goto fail;
        }
        if (ndef_file_load(ndef_msg_bufTEST, sizeof(ndef_msg_bufTEST)) < 0) {
		printk("Cannot load NDEF file!\n");
		goto fail;
	    }
    
        nfc_t2t_emulation_start();  // Start emulation again after payload is added
    }

fail:
#if CONFIG_REBOOT
    sys_reboot(SYS_REBOOT_COLD);
#endif /* CONFIG_REBOOT */

    return -EIO;
}
