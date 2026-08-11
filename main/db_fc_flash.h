/*
 * db_fc_flash - reflash the ArduPilot flight controller from the companion,
 * over USB OTG, with no PC involved.
 *
 * Proven end-to-end on the bench 10-08-2026 before any of this was written:
 * the ESP32-S3 hosted the FC's CDC-ACM port, commanded it into its bootloader,
 * erased, programmed 1,343,920 bytes at 117 KB/s and verified the CRC. Three
 * protocol/lifecycle traps found during that work are guarded and documented
 * directly in db_fc_flash.c.
 *
 * DESIGN NOTES worth knowing before changing anything here
 *
 *  - The image is stored in the `logs` FAT partition and flashed FROM LOCAL
 *    FLASH, never streamed from the browser. A dropped Wi-Fi association during
 *    a straight-through flash would leave the FC erased with a partial image;
 *    decoupling the network from the erase-and-program removes that failure
 *    mode entirely, and lets the boat be reflashed from a phone with no PC.
 *
 *  - The .apj is decoded ONCE at upload time into a raw .bin plus a sidecar
 *    holding board_id, size and the expected CRC. The expensive, fallible work
 *    happens while nothing is at stake, not during the time-critical flash.
 *
 *  - USB host and the FC UART on TX12/RX14 are INDEPENDENT paths. This module
 *    owns only the USB OTG peripheral (GPIO19/20). It must not disturb the
 *    existing MAVLink serial link.
 */

#ifndef DB_FC_FLASH_H
#define DB_FC_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/*
 * Where uploaded firmware lives, inside the sonar log's FAT mount.
 *
 * *** THESE NAMES MUST BE 8.3 *** The build sets CONFIG_FATFS_LFN_NONE, so
 * FATFS accepts at most 8 characters plus a 3-character extension. The first
 * attempt used "fc_firmware.bin" and "fc_firmware.json" and every fopen() failed
 * with nothing but "cannot open ... for writing" - an 11-character stem and a
 * 4-character extension are both illegal. `sonar.log` fits, which is why nothing
 * else in this project ever hit it. Enabling long filenames would change the
 * filesystem configuration for the sonar log too; renaming costs nothing.
 */
#define DB_FC_FLASH_DIR          "/logs"
#define DB_FC_FLASH_IMAGE_PATH   DB_FC_FLASH_DIR "/fcfw.bin"
#define DB_FC_FLASH_META_PATH    DB_FC_FLASH_DIR "/fcfw.jsn"

/*
 * Space set aside in `logs` for one firmware slot. db_sonar_log.c subtracts
 * this from the size it allows its own log file to reach, so the two cannot
 * starve each other. 1.5 MiB holds the 1.34 MB ArduRover image with headroom.
 * CHANGE BOTH TOGETHER: this constant and DB_SONAR_LOG_RESERVED_BYTES's use.
 */
#define DB_FC_FLASH_RESERVE_BYTES (1536u * 1024u)

/* Hard ceiling on an accepted image - also the STM32H743 app region size. */
#define DB_FC_FLASH_MAX_IMAGE_BYTES (1703936u)

typedef enum {
    DB_FC_FLASH_IDLE = 0,
    DB_FC_FLASH_WAITING_FOR_FC,   /* USB host up, no device yet */
    DB_FC_FLASH_REBOOTING,        /* MAVLink reboot-to-bootloader sent */
    DB_FC_FLASH_ERASING,
    DB_FC_FLASH_PROGRAMMING,
    DB_FC_FLASH_VERIFYING,
    DB_FC_FLASH_DONE,
    DB_FC_FLASH_FAILED,
} db_fc_flash_state_t;

typedef struct {
    db_fc_flash_state_t state;
    bool     usb_host_running;
    bool     fc_present;          /* a 0x1209:0x5741 CDC device is open */
    bool     in_bootloader;       /* it answered GET_SYNC with INSYNC/OK */
    uint32_t bl_rev;
    uint32_t board_id;            /* read from the FC bootloader */
    uint32_t fc_flash_size;
    uint32_t bytes_total;
    uint32_t bytes_done;
    uint32_t crc_expected;
    uint32_t crc_actual;
    char     last_error[128];
} db_fc_flash_status_t;

/* Metadata for the stored image, written at upload time. */
typedef struct {
    bool     present;
    uint32_t board_id;
    uint32_t image_size;
    uint32_t crc;                 /* ArduPilot variant, NOT zlib - see .c */
    char     source_name[64];     /* original .apj filename */
    char     description[64];     /* from the .apj, e.g. "STM32H743xx board" */
} db_fc_flash_image_info_t;

esp_err_t db_fc_flash_init(void);

/* Console. The web page polls this; a ring buffer avoids holding an httpd
 * worker open for the whole flash, which SSE would require. */
void   db_fc_flash_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
size_t db_fc_flash_console_read(uint32_t since_seq, char *out, size_t out_len,
                                uint32_t *next_seq);

void      db_fc_flash_get_status(db_fc_flash_status_t *out);
esp_err_t db_fc_flash_get_image_info(db_fc_flash_image_info_t *out);
esp_err_t db_fc_flash_delete_image(void);

/*
 * Decode an uploaded .apj into DB_FC_FLASH_IMAGE_PATH plus its sidecar.
 * Feed it the upload in chunks; call begin once, feed repeatedly, then finish.
 */
esp_err_t db_fc_flash_upload_begin(const char *filename, uint32_t board_id);
esp_err_t db_fc_flash_upload_write(const uint8_t *data, size_t len);
esp_err_t db_fc_flash_upload_finish(db_fc_flash_image_info_t *out);
void      db_fc_flash_upload_abort(void);

/*
 * Start the flash. Runs on its own task and returns immediately; watch
 * db_fc_flash_get_status() and the console.
 *
 * Refuses unless: an image is stored and its sidecar validates, the FC is
 * reachable, and the FC bootloader's board_id matches the image's. Refuses
 * outright if the vehicle is armed.
 */
esp_err_t db_fc_flash_start(void);

/* True if the flight controller reports itself armed - a hard block. */
bool db_fc_flash_vehicle_is_armed(void);

#endif /* DB_FC_FLASH_H */
