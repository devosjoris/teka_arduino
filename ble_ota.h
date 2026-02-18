#ifndef BLE_OTA_H
#define BLE_OTA_H

#include <Arduino.h>

class SFE_ST25DV64KC;

// ============================================================================
// BLE OTA Protocol — NFC-triggered, BLE data transfer
// ============================================================================
// The app writes a single 4-byte command ('OTAB') to the ST25DV NFC EEPROM
// telling the ESP32 to enable its BLE radio.  The ESP32 then advertises a
// BLE GATT OTA service and the app connects to stream firmware data.
// See NFC_OTA_PROTOCOL.md for the full specification.

// ---- NFC trigger (EEPROM address, shared with the old NFC-only OTA) --------
#define NFC_OTA_CMD_ADDR         0x78         // 4-byte command field
#define NFC_OTA_CMD_NONE         0x00000000
#define NFC_OTA_CMD_BLE_ENABLE   0x4F544142   // 'OTAB' — enable BLE for OTA

// ---- BLE GATT UUIDs -------------------------------------------------------
#define BLE_OTA_SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_OTA_CTRL_CHAR_UUID   "d5875408-fa51-4763-a75d-7d33cecebc31"
#define BLE_OTA_DATA_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ---- BLE control commands (app → ESP32, written to OTA Control char) -------
#define BLE_OTA_CTRL_START       0x01   // + 4 bytes LE firmware size
#define BLE_OTA_CTRL_END         0x02
#define BLE_OTA_CTRL_ABORT       0x03

// ---- BLE status notifications (ESP32 → app, on OTA Control char) -----------
#define BLE_OTA_STATUS_READY     0x01
#define BLE_OTA_STATUS_ERROR     0x02
#define BLE_OTA_STATUS_DONE      0x03

// ---- Timeouts --------------------------------------------------------------
#define BLE_OTA_ADV_TIMEOUT_MS   300000   // 5 minutes

// ---- State machine ---------------------------------------------------------
typedef enum {
    BLE_OTA_STATE_IDLE,
    BLE_OTA_STATE_ADVERTISING,
    BLE_OTA_STATE_CONNECTED,
    BLE_OTA_STATE_RECEIVING,
    BLE_OTA_STATE_DONE
} ble_ota_state_t;

// Initialise the BLE-OTA module.  Pass the NFC tag pointer so we can poll
// the EEPROM for the 'OTAB' trigger.  Call once, after setup_tag().
void ble_ota_setup(SFE_ST25DV64KC *tag);

// Call from loop().  Returns true while OTA is active (advertising, connected
// or transferring) — the caller should skip normal application work.
bool ble_ota_poll();

// Returns true if a firmware transfer is actively in progress.
bool ble_ota_in_progress();

#endif // BLE_OTA_H
