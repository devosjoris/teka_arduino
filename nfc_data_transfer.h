#ifndef NFC_DATA_TRANSFER_H
#define NFC_DATA_TRANSFER_H

#include <Arduino.h>
#include <stdint.h>

class SFE_ST25DV64KC;

// ============================================================================
// NFC Data Transfer Protocol
// ============================================================================
// Protocol for transferring sensor log data from ESP32 filesystem to app via NFC.
//
// Tracking mechanism:
// Each log entry has a flags byte where:
//   - Bit 0 (SENSLOG_FLAG_RTC_VALID): RTC was valid when logged
//   - Bit 1 (SENSLOG_FLAG_READOUT_DONE): Entry has been read out via NFC
//
// Memory layout in NFC tag (all little-endian, 4-byte aligned):
//
//   Offset  Size  Field
//   ------  ----  -----
//   0x60    4     CMD_FIELD: Command from app (see NFC_DT_CMD_*)
//   0x64    4     STATUS_FIELD: Status from ESP32 (see NFC_DT_STATUS_*)
//   0x68    4     FIELD_COUNT: Number of log entries in this batch
//   0x6C    4     TOTAL_PENDING: Total entries still pending readout
//   0x70    4     CRC32: CRC32 of data payload
//   0x74    4     DATA_LENGTH: Length of data payload in bytes
//   0xC8    ...   DATA_PAYLOAD: Log entry data
//
// Each log entry in DATA_PAYLOAD (12 bytes, packed):
//   uint32_t sensorValue;
//   uint32_t unixTimestamp;
//   uint8_t  flags;       // bit0=rtcValid, bit1=readoutDone
//   uint8_t  _pad[3];
//
// Flow:
// 1. App writes NFC_DT_CMD_REQUEST_DATA to CMD_FIELD
// 2. ESP32 scans ring buffer for entries without READOUT_DONE flag
// 3. ESP32 copies up to 512 entries to NFC tag, writes CRC32
// 4. ESP32 writes NFC_DT_STATUS_DATA_READY
// 5. App reads data, verifies CRC32
// 6. App writes NFC_DT_CMD_ACK_DATA if CRC valid
// 7. ESP32 marks those entries with READOUT_DONE flag in filesystem
// ============================================================================

// Base address for data transfer protocol
// Memory map: 0x00-0x5C = settings, 0x60-0xC4 = DT protocol, 0xC8+ = legacy data
#define NFC_DT_BASE_ADDR        0x60  // 96 decimal

// Command field (written by app)
#define NFC_DT_CMD_ADDR         (NFC_DT_BASE_ADDR + 0x00)  // 0x60
// Status field (written by ESP32)
#define NFC_DT_STATUS_ADDR      (NFC_DT_BASE_ADDR + 0x04)  // 0x64
// Number of entries in this batch
#define NFC_DT_FIELD_COUNT_ADDR (NFC_DT_BASE_ADDR + 0x08)  // 0x68
// Total entries still pending readout (for app to know if more batches needed)
#define NFC_DT_TOTAL_PENDING_ADDR (NFC_DT_BASE_ADDR + 0x0C)  // 0x6C
// CRC32 of data payload
#define NFC_DT_CRC32_ADDR       (NFC_DT_BASE_ADDR + 0x10)  // 0x70
// Length of data payload in bytes
#define NFC_DT_DATA_LEN_ADDR    (NFC_DT_BASE_ADDR + 0x14)  // 0x74
// Start of data payload (after the MEM_VAL_DATA_START at 0xC8)
#define NFC_DT_DATA_START_ADDR  0xC8  // 200 decimal, same as MEM_VAL_DATA_START

// Each log entry is 12 bytes (same as senslog_fs LogEntry)
#define NFC_DT_ENTRY_SIZE       12

// Maximum entries per transfer batch (64 log entries)
#define NFC_DT_MAX_ENTRIES      64 

// Maximum data payload size per transfer batch (512 entries * 12 bytes = 6144 bytes)
#define NFC_DT_MAX_DATA_SIZE    (NFC_DT_MAX_ENTRIES * NFC_DT_ENTRY_SIZE)

// Commands from app (written to CMD_FIELD)
#define NFC_DT_CMD_NONE         0x00000000  // No command / idle
#define NFC_DT_CMD_REQUEST_DATA 0x52455144  // 'DREQ' - Request sensor data
#define NFC_DT_CMD_ACK_DATA     0x41434B44  // 'DACK' - Acknowledge data received & CRC valid
#define NFC_DT_CMD_NACK_DATA    0x4E41434B  // 'NACK' - CRC mismatch, resend
#define NFC_DT_CMD_RESET_FLAGS  0x52535446  // 'RSTF' - Clear all READOUT_DONE flags (re-read all)

// Status from ESP32 (written to STATUS_FIELD)
#define NFC_DT_STATUS_IDLE          0x00000000  // Idle, ready for commands
#define NFC_DT_STATUS_DATA_READY    0x44524459  // 'DRDY' - Data ready to read
#define NFC_DT_STATUS_ACK_OK        0x41434F4B  // 'ACOK' - ACK processed, pointer advanced
#define NFC_DT_STATUS_NO_DATA       0x4E444154  // 'NDAT' - No new data available
#define NFC_DT_STATUS_ERROR         0x45525221  // 'ERR!' - Error occurred
#define NFC_DT_STATUS_BUSY          0x42555359  // 'BUSY' - Processing command

// Packed log entry structure (must match senslog_fs LogEntry)
#pragma pack(push, 1)
typedef struct {
    uint32_t sensorValue;
    uint32_t unixTimestamp;
    uint8_t  flags;       // bit0=rtcValid, bit1=readoutDone
    uint8_t  _pad[3];
} NfcDtLogEntry;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the data transfer module
// Must be called after tag is initialized
void nfc_dt_init(SFE_ST25DV64KC* tag);

// Poll for commands from app and process them
// Call this periodically from main loop
// Returns true if a command was processed
bool nfc_dt_poll(void);

// Get number of entries pending (not yet read by app, i.e., without READOUT_DONE flag)
uint16_t nfc_dt_get_pending_count(void);

// Clear all READOUT_DONE flags (to allow re-reading all data)
void nfc_dt_reset_all_flags(void);

#ifdef __cplusplus
}
#endif

#endif // NFC_DATA_TRANSFER_H
