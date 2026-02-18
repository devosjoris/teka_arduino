#include "ble_ota.h"
#include "tag_support.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#if defined(ESP32)
#include <Update.h>
#include <esp32s3/rom/miniz.h>          // tinfl decompressor in ROM
#endif

// ============================================================================
// BLE OTA — NFC-triggered, BLE data transfer
// ============================================================================
// 1. App writes 'OTAB' (0x4F544142) to NFC EEPROM address 0x78.
// 2. ESP32 sees the trigger, clears it, and starts BLE advertising.
// 3. App connects via BLE and sends START / data / END commands.
// 4. ESP32 writes firmware to OTA partition and reboots.
//
// Gzip-compressed firmware is auto-detected (magic 0x1F 0x8B) and
// decompressed on the fly using the ESP32-S3 ROM tinfl library.
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

// ---- Gzip streaming decompression ----------------------------------------
#if defined(ESP32)
static bool                s_isGzip        = false;
static bool                s_gzipHdrParsed = false;
static uint8_t             s_gzipHdr[10];
static uint8_t             s_gzipHdrLen    = 0;
static tinfl_decompressor  s_inflator;
static uint8_t             s_dict[TINFL_LZ_DICT_SIZE];   // 32 KB sliding window
static size_t              s_dictOfs       = 0;
#endif

// ---- Helpers --------------------------------------------------------------

static void reset_gzip_state()
{
#if defined(ESP32)
    s_isGzip        = false;
    s_gzipHdrParsed = false;
    s_gzipHdrLen    = 0;
    s_dictOfs       = 0;
#endif
}

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

#if defined(ESP32)
    // ---- First bytes: detect gzip -----------------------------------------
    if (s_received == 0 && len >= 2 &&
        data[0] == 0x1F && data[1] == 0x8B)
    {
        Serial.println("BLE OTA: gzip detected");
        s_isGzip        = true;
        s_gzipHdrParsed = false;
        s_gzipHdrLen    = 0;
        s_dictOfs       = 0;
        tinfl_init(&s_inflator);
    }

    if (s_isGzip) {
        const uint8_t *src = data;
        size_t srcLeft = len;

        // 1) Parse the 10-byte gzip header (basic gzip only)
        if (!s_gzipHdrParsed) {
            while (srcLeft > 0 && s_gzipHdrLen < 10) {
                s_gzipHdr[s_gzipHdrLen++] = *src++;
                srcLeft--;
            }
            if (s_gzipHdrLen == 10) {
                if (s_gzipHdr[0] != 0x1F || s_gzipHdr[1] != 0x8B ||
                    s_gzipHdr[2] != 0x08)
                {
                    s_dataError = true;
                    return false;
                }
                uint8_t flg = s_gzipHdr[3];
                if (flg & 0x1C) {   // FEXTRA | FNAME | FCOMMENT
                    s_dataError = true;
                    return false;
                }
                s_gzipHdrParsed = true;
                Serial.println("BLE OTA: gzip header OK");
            }
        }

        // 2) Decompress remaining bytes with tinfl (raw deflate)
        while (srcLeft > 0 && s_gzipHdrParsed) {
            size_t inBytes  = srcLeft;
            size_t outBytes = TINFL_LZ_DICT_SIZE - s_dictOfs;

            tinfl_status st = tinfl_decompress(
                &s_inflator, src, &inBytes,
                s_dict, s_dict + s_dictOfs, &outBytes,
                TINFL_FLAG_HAS_MORE_INPUT);

            if (st < TINFL_STATUS_DONE) {
                Serial.printf("BLE OTA: tinfl error %d\n", (int)st);
                s_dataError = true;
                return false;
            }

            src     += inBytes;
            srcLeft -= inBytes;

            if (outBytes > 0) {
                if (Update.write(s_dict + s_dictOfs, outBytes) != outBytes) {
                    Serial.printf("BLE OTA: flash write failed: %s\n",
                                  Update.errorString());
                    s_dataError = true;
                    return false;
                }
            }

            s_dictOfs = (s_dictOfs + outBytes) & (TINFL_LZ_DICT_SIZE - 1);
            if (st == TINFL_STATUS_DONE) break;
        }
    } else {
        // ---- Raw (uncompressed) path --------------------------------------
        if (Update.write((uint8_t *)data, len) != len) {
            Serial.printf("BLE OTA: flash write failed: %s\n",
                          Update.errorString());
            s_dataError = true;
            return false;
        }
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
                s_pendingFwSize = (uint8_t)v[1]        |
                                  ((uint8_t)v[2] << 8)  |
                                  ((uint8_t)v[3] << 16) |
                                  ((uint8_t)v[4] << 24);
                s_flagStart = true;
            }
            break;
        case BLE_OTA_CTRL_END:   s_flagEnd   = true; break;
        case BLE_OTA_CTRL_ABORT: s_flagAbort = true; break;
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
        BLEDevice::init("DustSensor-OTA");
        BLEDevice::setMTU(517);

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

        // Data characteristic (write without response)
        s_dataChar = svc->createCharacteristic(
            BLE_OTA_DATA_CHAR_UUID,
            BLECharacteristic::PROPERTY_WRITE_NR);
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
            reset_gzip_state();
            s_otaBegun = false;
            Serial.println("BLE OTA: disconnected during transfer — aborted");
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
        reset_gzip_state();
        s_otaBegun  = false;
        s_totalSize = 0;
        s_received  = 0;
        s_dataError = false;
        notify_status(BLE_OTA_STATUS_ERROR);
        s_state = BLE_OTA_STATE_CONNECTED;   // stay connected for retry
        return true;
    }

    // ---- Handle START command ---------------------------------------------
    if (s_flagStart) {
        s_flagStart = false;
        uint32_t fwSize = s_pendingFwSize;

        Serial.printf("BLE OTA: START — firmware size = %u bytes\n", fwSize);

        if (fwSize == 0) {
            notify_status(BLE_OTA_STATUS_ERROR);
            return true;
        }

#if defined(ESP32)
        // Use UPDATE_SIZE_UNKNOWN — the size may be the compressed size
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
            Serial.printf("BLE OTA: Update.begin failed: %s\n",
                          Update.errorString());
            notify_status(BLE_OTA_STATUS_ERROR);
            return true;
        }
#endif

        s_totalSize = fwSize;
        s_received  = 0;
        s_otaBegun  = true;
        s_dataError = false;
        reset_gzip_state();

        s_state = BLE_OTA_STATE_RECEIVING;
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
        reset_gzip_state();
        s_otaBegun  = false;
        s_dataError = false;
        s_state     = BLE_OTA_STATE_CONNECTED;
        notify_status(BLE_OTA_STATUS_ERROR);
        return true;
    }

    // ---- Handle END command -----------------------------------------------
    if (s_flagEnd) {
        s_flagEnd = false;

        Serial.printf("BLE OTA: END — received %u / %u bytes\n",
                       s_received, s_totalSize);

#if defined(ESP32)
        reset_gzip_state();

        if (!Update.end(true)) {
            Serial.printf("BLE OTA: Update.end failed: %s\n",
                          Update.errorString());
            notify_status(BLE_OTA_STATUS_ERROR);
            s_otaBegun = false;
            s_state    = BLE_OTA_STATE_CONNECTED;
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
