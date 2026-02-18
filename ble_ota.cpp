#include "ble_ota.h"
#include "tag_support.h"
#include "pinmap.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#if defined(ESP32)
#include <Update.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

// ============================================================================
// BLE OTA — NFC-triggered, BLE data transfer
// ============================================================================
// 1. App writes 'OTAB' (0x4F544142) to NFC EEPROM address 0x78.
// 2. ESP32 sees the trigger, clears it, and starts BLE advertising.
// 3. App connects via BLE and sends START / data / END commands.
// 4. ESP32 writes firmware to OTA partition and reboots.
// ============================================================================

namespace {

// ---- References & state ---------------------------------------------------
static SFE_ST25DV64KC *s_tag   = nullptr;
static ble_ota_state_t s_state = BLE_OTA_STATE_IDLE;

// ---- BLE objects ----------------------------------------------------------
static BLEServer         *s_server   = nullptr;
static BLECharacteristic *s_ctrlChar = nullptr;
static BLECharacteristic *s_dataChar = nullptr;
static bool               s_bleInit  = false;
static bool               s_connected = false;
static uint32_t           s_advStartMs = 0;

// ---- OTA transfer state ---------------------------------------------------
static uint32_t s_totalSize  = 0;
static uint32_t s_received   = 0;
static bool     s_otaBegun   = false;

// ---- Flags set by BLE callbacks, consumed in poll() -----------------------
static volatile bool     s_flagStart      = false;
static volatile uint32_t s_pendingFwSize  = 0;
static volatile bool     s_flagEnd        = false;
static volatile bool     s_flagAbort      = false;
static volatile bool     s_flagDisconnect = false;
static volatile bool     s_dataError      = false;

// ---- Helpers --------------------------------------------------------------

static void notify_status(uint8_t status)
{
    if (s_ctrlChar && s_connected) {
        s_ctrlChar->setValue(&status, 1);
        s_ctrlChar->notify();
    }
}

// ---------------------------------------------------------------------------
// Process a block of firmware bytes.
// Called from the BLE data-write callback (BTC task).
// Sets s_dataError on failure so poll() can clean up.
// ---------------------------------------------------------------------------
static bool process_fw_data(const uint8_t *data, size_t len)
{
    if (!s_otaBegun || s_dataError) return false;

    // Debug: log first chunk and every 100th to track sizes
    static uint32_t s_chunkCount = 0;
    if (s_chunkCount == 0) {
        Serial.printf("BLE OTA: first chunk %u bytes, magic=0x%02X 0x%02X\n",
                      len, data[0], len > 1 ? data[1] : 0);
    }
    s_chunkCount++;

#if defined(ESP32)
    if (Update.write((uint8_t *)data, len) != len) {
        Serial.printf("BLE OTA: flash write failed: %s\n",
                      Update.errorString());
        s_dataError = true;
        return false;
    }
#endif

    s_received += len;
    return true;
}

// ---- BLE callbacks --------------------------------------------------------

class ServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        s_connected = true;
        Serial.println("BLE OTA: client connected");
    }
    void onDisconnect(BLEServer *) override {
        s_connected      = false;
        s_flagDisconnect = true;
        Serial.println("BLE OTA: client disconnected");
    }
};

class CtrlCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *chr) override {
        std::string v = chr->getValue();
        if (v.empty()) return;

        switch ((uint8_t)v[0]) {
        case BLE_OTA_CTRL_START:
            if (v.size() >= 5) {
                uint32_t fwSize = (uint8_t)v[1]        |
                                  ((uint8_t)v[2] << 8)  |
                                  ((uint8_t)v[3] << 16) |
                                  ((uint8_t)v[4] << 24);
                s_pendingFwSize = fwSize;

                // Begin the Update session *synchronously* in the BLE
                // callback so that s_otaBegun is true before any data
                // chunks arrive (they run on the same BTC task).
#if defined(ESP32)
                if (s_otaBegun && Update.isRunning()) {
                    Update.abort();
                    s_otaBegun = false;
                }
                if (fwSize > 0 && Update.begin(fwSize, U_FLASH)) {
                    s_otaBegun  = true;
                    s_received  = 0;
                    s_totalSize = fwSize;
                    s_dataError = false;
                }
#endif
                s_flagStart = true;
            }
            break;
        case BLE_OTA_CTRL_END:   s_flagEnd   = true; break;
        case BLE_OTA_CTRL_ABORT: s_flagAbort = true; break;
        default:
            // Ignore unknown control commands
            break;
        }
    }
};

class DataCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *chr) override {
        std::string v = chr->getValue();
        if (!v.empty())
            process_fw_data((const uint8_t *)v.data(), v.size());
    }
};

static ServerCB s_serverCB;
static CtrlCB   s_ctrlCB;
static DataCB   s_dataCB;

// ---- BLE lifecycle --------------------------------------------------------

static void start_ble()
{
    if (!s_bleInit) {
        Serial.println("BLE OTA: initialising BLE...");
        Serial.flush();

#if defined(ESP32)
        // The BLE radio draws a large current spike on init.
        // Temporarily disable the brownout detector to survive the transient.
        CLEAR_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
#endif

        BLEDevice::init("DustSensor-OTA");

#if defined(ESP32)
        // Re-enable brownout detector now that BLE is up.
        SET_PERI_REG_MASK(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
#endif

        BLEDevice::setMTU(512);

        s_server = BLEDevice::createServer();
        s_server->setCallbacks(&s_serverCB);

        BLEService *svc = s_server->createService(BLE_OTA_SERVICE_UUID);

        // Control characteristic (write + notify)
        s_ctrlChar = svc->createCharacteristic(
            BLE_OTA_CTRL_CHAR_UUID,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY);
        s_ctrlChar->addDescriptor(new BLE2902());
        s_ctrlChar->setCallbacks(&s_ctrlCB);    

        // Data characteristic (write with response)
        s_dataChar = svc->createCharacteristic(
            BLE_OTA_DATA_CHAR_UUID,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        s_dataChar->setCallbacks(&s_dataCB);

        svc->start();

        BLEAdvertising *adv = BLEDevice::getAdvertising();
        adv->addServiceUUID(BLE_OTA_SERVICE_UUID);
        adv->setScanResponse(true);
        adv->setMinPreferred(0x06);   // helps iPhone connection
        adv->setMinPreferred(0x12);

        s_bleInit = true;
    }

    BLEDevice::startAdvertising();
    s_state      = BLE_OTA_STATE_ADVERTISING;
    s_advStartMs = millis();
    Serial.println("BLE OTA: advertising started (5 min timeout)");
}

static void stop_ble()
{
    if (!s_bleInit) return;

    // Fully deinitialise BLE so the ESP32 can enter light-sleep cleanly.
    BLEDevice::deinit(true);
    s_bleInit  = false;
    s_server   = nullptr;
    s_ctrlChar = nullptr;
    s_dataChar = nullptr;
    s_state    = BLE_OTA_STATE_IDLE;
    Serial.println("BLE OTA: BLE deinitialised");
}

} // anonymous namespace

// ============================================================================
// Public API
// ============================================================================

void ble_ota_setup(SFE_ST25DV64KC *tag)
{
    s_tag = tag;

    // Clear any stale trigger left in NFC EEPROM
    write_int_tag(s_tag, NFC_OTA_CMD_ADDR, NFC_OTA_CMD_NONE);

    Serial.println("BLE OTA: ready (NFC-triggered BLE protocol)");
}

bool ble_ota_poll()
{
    // ---- IDLE: check NFC EEPROM for 'OTAB' trigger ------------------------
    if (s_state == BLE_OTA_STATE_IDLE && s_tag) {
        uint32_t cmd = read_int_tag(s_tag, NFC_OTA_CMD_ADDR);
        if (cmd == NFC_OTA_CMD_BLE_ENABLE) {
            Serial.println("BLE OTA: NFC trigger 'OTAB' detected");
            write_int_tag(s_tag, NFC_OTA_CMD_ADDR, NFC_OTA_CMD_NONE);
            start_ble();
        }
    }

    // ---- ADVERTISING: wait for BLE connection or timeout ------------------
    if (s_state == BLE_OTA_STATE_ADVERTISING) {
        if (s_connected) {
            s_state = BLE_OTA_STATE_CONNECTED;
            Serial.println("BLE OTA: connected — waiting for START command");
        } else if ((millis() - s_advStartMs) > BLE_OTA_ADV_TIMEOUT_MS) {
            Serial.println("BLE OTA: advertising timeout — returning to idle");
            stop_ble();
            return false;
        }
        return true;   // keep the device awake while advertising
    }

    // ---- Handle disconnect ------------------------------------------------
    if (s_flagDisconnect) {
        s_flagDisconnect = false;
        if (s_state == BLE_OTA_STATE_RECEIVING) {
#if defined(ESP32)
            if (s_otaBegun && Update.isRunning()) Update.abort();
#endif
            s_otaBegun = false;
            Serial.println("BLE OTA: disconnected during transfer — rebooting...");
            stop_ble();
            delay(200);
            ESP.restart();
        }
        stop_ble();
        return false;
    }

    // ---- Handle ABORT command ---------------------------------------------
    if (s_flagAbort) {
        s_flagAbort = false;
        Serial.println("BLE OTA: ABORT received");
#if defined(ESP32)
        if (s_otaBegun && Update.isRunning()) Update.abort();
#endif
        s_otaBegun  = false;
        s_totalSize = 0;
        s_received  = 0;
        s_dataError = false;
        notify_status(BLE_OTA_STATUS_ERROR);
        Serial.println("BLE OTA: ABORT — rebooting...");
        delay(200);
        ESP.restart();
        return true;
    }

    // ---- Handle START command ---------------------------------------------
    if (s_flagStart) {
        s_flagStart = false;
        uint32_t fwSize = s_pendingFwSize;

        Serial.printf("BLE OTA: START — firmware size = %u bytes\n", fwSize);

        if (fwSize == 0 || !s_otaBegun) {
            // Update.begin() was attempted in the BLE callback;
            // if it failed s_otaBegun is still false.
            Serial.println("BLE OTA: Update.begin failed (from callback)");
            notify_status(BLE_OTA_STATUS_ERROR);
            return true;
        }

        s_state = BLE_OTA_STATE_RECEIVING;
        // Red + blue LED on while receiving firmware
        digitalWrite(PIN_LED_R, HIGH);
        digitalWrite(PIN_LED_B, HIGH);
        digitalWrite(PIN_LED_G, LOW);
        notify_status(BLE_OTA_STATUS_READY);
        Serial.println("BLE OTA: READY — streaming firmware over BLE");
        return true;
    }

    // ---- Check for errors during data writes ------------------------------
    if (s_dataError && s_state == BLE_OTA_STATE_RECEIVING) {
        Serial.println("BLE OTA: error during data write");
#if defined(ESP32)
        if (Update.isRunning()) Update.abort();
#endif
        s_otaBegun  = false;
        s_dataError = false;
        notify_status(BLE_OTA_STATUS_ERROR);
        Serial.println("BLE OTA: data error — rebooting...");
        delay(200);
        ESP.restart();
        return true;
    }

    // ---- Handle END command -----------------------------------------------
    if (s_flagEnd) {
        s_flagEnd = false;

        Serial.printf("BLE OTA: END — received %u / %u bytes (avg chunk = %u)\n",
                       s_received, s_totalSize,
                       s_received > 0 ? s_received / ((s_received / 512) + 1) : 0);

#if defined(ESP32)
        if (!Update.end(true)) {
            Serial.printf("BLE OTA: Update.end failed: %s\n",
                          Update.errorString());
            notify_status(BLE_OTA_STATUS_ERROR);
            s_otaBegun = false;
            Serial.println("BLE OTA: verify failed — rebooting...");
            delay(200);
            ESP.restart();
            return true;
        }

        Serial.println("BLE OTA: firmware verified — rebooting...");
        s_state = BLE_OTA_STATE_DONE;
        notify_status(BLE_OTA_STATUS_DONE);
        delay(500);   // give BLE stack time to deliver the notification
        ESP.restart();
#endif
        return true;
    }

    // ---- Periodic progress logging (every 2 s) ----------------------------
    if (s_state == BLE_OTA_STATE_RECEIVING && s_received > 0) {
        static uint32_t s_lastLog = 0;
        uint32_t now = millis();
        if (now - s_lastLog > 2000) {
            Serial.printf("BLE OTA: %u / %u (%u%%)\n",
                          s_received, s_totalSize,
                          (s_received * 100) / s_totalSize);
            s_lastLog = now;
        }
    }

    return (s_state != BLE_OTA_STATE_IDLE);
}

bool ble_ota_in_progress()
{
    return (s_state == BLE_OTA_STATE_RECEIVING);
}
