#ifndef NFC_OTA_H
#define NFC_OTA_H

#include <Arduino.h>

class SFE_ST25DV64KC;

// Initializes the ST25DV mailbox for OTA-over-NFC.
// Call this after the tag has been initialized (e.g. after setup_tag()).
void nfc_ota_setup(SFE_ST25DV64KC *tag);

// Polls the ST25DV mailbox for OTA packets and applies them using the ESP32 Update API.
// Returns true if OTA is in progress (callers should skip normal app work when true).
bool nfc_ota_poll();

// Returns true if an OTA transfer has started and is not finished/aborted yet.
bool nfc_ota_in_progress();

#endif // NFC_OTA_H
