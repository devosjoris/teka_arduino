#ifndef NFC_OTA_H
#define NFC_OTA_H

#include <Arduino.h>

class SFE_ST25DV64KC;

// ============================================================================
// NFC OTA Protocol - Hybrid EEPROM + Mailbox
// ============================================================================
// Uses ST25DV EEPROM for handshaking (commands, status, metadata)
// Uses ST25DV Mailbox (FTM) for bulk data transfer - fast, no EEPROM wear

// Memory Map (EEPROM - Handshaking)
#define NFC_OTA_CMD_ADDR        0x78  // Command field (app -> ESP32)
#define NFC_OTA_STATUS_ADDR     0x7C  // Status field (ESP32 -> app)
#define NFC_OTA_CHUNK_NUM_ADDR  0x80  // Current chunk number (0-indexed)
#define NFC_OTA_TOTAL_SIZE_ADDR 0x84  // Total firmware size in bytes
#define NFC_OTA_CHUNK_SIZE_ADDR 0x88  // Current chunk size in bytes
#define NFC_OTA_CRC32_ADDR      0x8C  // CRC32 of current chunk data

// OTA Commands (app -> ESP32)
#define NFC_OTA_CMD_NONE        0x00000000
#define NFC_OTA_CMD_START       0x4F544153  // 'OTAS' - Start OTA transfer
#define NFC_OTA_CMD_DATA_START  0x4F544442  // 'OTDB' - Begin mailbox mode
#define NFC_OTA_CMD_DATA        0x4F544144  // 'OTAD' - Chunk ready in mailbox
#define NFC_OTA_CMD_END         0x4F544145  // 'OTAE' - End transfer
#define NFC_OTA_CMD_ABORT       0x4F544158  // 'OTAX' - Abort transfer

// OTA Status (ESP32 -> app)
#define NFC_OTA_STATUS_IDLE  0x00000000
#define NFC_OTA_STATUS_READY 0x4F545259  // 'OTRY' - Ready for next chunk
#define NFC_OTA_STATUS_BUSY  0x4F544259  // 'OTBY' - Currently processing
#define NFC_OTA_STATUS_ERROR 0x4F544552  // 'OTER' - Error occurred
#define NFC_OTA_STATUS_DONE  0x4F54444E  // 'OTDN' - OTA complete, rebooting

// Chunk size (mailbox size = 256 bytes)
#define NFC_OTA_MAX_CHUNK_SIZE 256

// OTA State Machine
typedef enum {
  OTA_STATE_IDLE,
  OTA_STATE_PREPARING,
  OTA_STATE_MAILBOX_MODE,
  OTA_STATE_DONE
} nfc_ota_state_t;

// Initializes the OTA module with a reference to the NFC tag.
// Call this after the tag has been initialized (e.g. after setup_tag()).
void nfc_ota_setup(SFE_ST25DV64KC *tag);

// Polls the NFC EEPROM for OTA commands and processes them.
// Returns true if OTA is in progress (callers should skip normal app work when true).
bool nfc_ota_poll();

// Returns true if an OTA transfer has started and is not finished/aborted yet.
bool nfc_ota_in_progress();

#endif // NFC_OTA_H
