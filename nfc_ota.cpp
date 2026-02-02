#include "nfc_ota.h"
#include "tag_support.h"

#include <Wire.h>

#include "SparkFun_ST25DV64KC_Arduino_Library.h"

#if defined(ESP32)
#include <Update.h>
#endif

// ============================================================================
// NFC OTA - Hybrid EEPROM + Mailbox protocol
// ============================================================================
// EEPROM for handshaking (commands, status, metadata)
// Mailbox (FTM) for bulk data transfer - fast, no EEPROM wear

namespace {

static SFE_ST25DV64KC *s_tag = nullptr;

static nfc_ota_state_t s_state = OTA_STATE_IDLE;
static uint32_t s_totalSize = 0;
static uint32_t s_received = 0;
static uint32_t s_expectedChunk = 0;

// Buffer for reading chunk data from mailbox
static uint8_t s_chunkBuffer[NFC_OTA_MAX_CHUNK_SIZE];

// CRC32 (Ethernet polynomial, same as zlib)
static uint32_t crc32_calc(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xEDB88320u;
      else
        crc >>= 1;
    }
  }
  return crc ^ 0xFFFFFFFF;
}

// Enable mailbox (Fast Transfer Mode)
static bool mailbox_enable()
{
  if (s_tag == nullptr)
    return false;

  bool ok = true;
  // Enable mailbox mode in FTM register (system)
  ok &= s_tag->st25_io.setRegisterBit(SF_ST25DV64KC_ADDRESS::SYSTEM, REG_FTM, BIT_FTM_MB_MODE);
  // Enable mailbox in dynamic register
  ok &= s_tag->st25_io.setRegisterBit(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, BIT_MB_CTRL_DYN_MB_EN);
  return ok;
}

// Disable mailbox
static void mailbox_disable()
{
  if (s_tag == nullptr)
    return;

  // Disable mailbox in dynamic register
  s_tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, 0x00);
  
  // Disable FTM mode in system register
  uint8_t ftm = 0;
  if (s_tag->st25_io.readSingleByte(SF_ST25DV64KC_ADDRESS::SYSTEM, REG_FTM, &ftm))
  {
    ftm = (uint8_t)(ftm & (uint8_t)~BIT_FTM_MB_MODE);
    s_tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::SYSTEM, REG_FTM, ftm);
  }
}

// Read data from mailbox
static bool mailbox_read(uint8_t *out, uint16_t expectedLen, uint16_t *actualLen)
{
  *actualLen = 0;
  if (s_tag == nullptr)
    return false;

  // Check mailbox control register
  uint8_t ctrl = 0;
  if (!s_tag->st25_io.readSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, &ctrl))
    return false;

  // Check if mailbox is enabled
  if ((ctrl & BIT_MB_CTRL_DYN_MB_EN) == 0)
  {
    if (!mailbox_enable())
      return false;
    if (!s_tag->st25_io.readSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, &ctrl))
      return false;
  }

  // Check if RF has put a message (RF_PUT_MSG flag)
  if ((ctrl & BIT_MB_CTRL_DYN_RF_PUT_MSG) == 0)
    return false; // No new message

  // Read mailbox length
  uint8_t len8 = 0;
  if (!s_tag->st25_io.readSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_LEN_DYN, &len8))
    return false;

  uint16_t len = len8 + 1; // MB_LEN_DYN is length - 1
  
  if (len == 0)
  {
    // Clear mailbox anyway
    s_tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, BIT_MB_CTRL_DYN_MB_EN);
    return true;
  }

  // Validate length
  if (len > NFC_OTA_MAX_CHUNK_SIZE || len > expectedLen)
  {
    Serial.print("NFC OTA: mailbox len mismatch, got ");
    Serial.print(len);
    Serial.print(" expected ");
    Serial.println(expectedLen);
    // Clear mailbox
    s_tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, BIT_MB_CTRL_DYN_MB_EN);
    return false;
  }

  // Read mailbox data
  if (!s_tag->st25_io.readMultipleBytes(SF_ST25DV64KC_ADDRESS::DATA, MAILBOX_BASE, out, len))
    return false;

  *actualLen = len;

  // Clear mailbox (acknowledge)
  s_tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, BIT_MB_CTRL_DYN_MB_EN);

  return true;
}

static void write_status(uint32_t status)
{
  write_int_tag(s_tag, NFC_OTA_STATUS_ADDR, status);
}

static void clear_command()
{
  write_int_tag(s_tag, NFC_OTA_CMD_ADDR, NFC_OTA_CMD_NONE);
}

static void ota_abort(const char *reason)
{
  Serial.print("NFC OTA abort: ");
  Serial.println(reason);

#if defined(ESP32)
  if (Update.isRunning())
    Update.abort();
#endif

  // Disable mailbox
  mailbox_disable();

  s_state = OTA_STATE_IDLE;
  s_totalSize = 0;
  s_received = 0;
  s_expectedChunk = 0;

  write_status(NFC_OTA_STATUS_ERROR);
  clear_command();
}

static void handle_start()
{
  Serial.println("NFC OTA: CMD_START received");

  // Read total firmware size
  uint32_t totalSize = read_int_tag(s_tag, NFC_OTA_TOTAL_SIZE_ADDR);
  
  if (totalSize == 0)
  {
    ota_abort("size=0");
    return;
  }

  Serial.print("NFC OTA start, size=");
  Serial.println(totalSize);

#if defined(ESP32)
  if (!Update.begin(totalSize, U_FLASH))
  {
    Serial.print("Update.begin failed: ");
    Serial.println(Update.errorString());
    ota_abort("Update.begin failed");
    return;
  }
#else
  ota_abort("not ESP32");
  return;
#endif

  s_state = OTA_STATE_PREPARING;
  s_totalSize = totalSize;
  s_received = 0;
  s_expectedChunk = 0;

  // Signal ready
  write_status(NFC_OTA_STATUS_READY);
  clear_command();

  Serial.println("NFC OTA: Prepared, waiting for DATA_START");
}

static void handle_data_start()
{
  if (s_state != OTA_STATE_PREPARING)
  {
    Serial.println("NFC OTA: DATA_START in wrong state");
    ota_abort("wrong state");
    return;
  }

  Serial.println("NFC OTA: CMD_DATA_START received, enabling mailbox");

  // Enable Fast Transfer Mode (mailbox)
  if (!mailbox_enable())
  {
    ota_abort("mailbox enable failed");
    return;
  }

  s_state = OTA_STATE_MAILBOX_MODE;

  write_status(NFC_OTA_STATUS_READY);
  clear_command();

  Serial.println("NFC OTA: Mailbox mode active, ready for chunks");
}

static void handle_data()
{
  if (s_state != OTA_STATE_MAILBOX_MODE)
  {
    Serial.println("NFC OTA: DATA in wrong state");
    clear_command();
    return;
  }

  // Set busy status while processing
  write_status(NFC_OTA_STATUS_BUSY);

  // Read chunk metadata from EEPROM
  uint32_t chunkNum = read_int_tag(s_tag, NFC_OTA_CHUNK_NUM_ADDR);
  uint32_t chunkSize = read_int_tag(s_tag, NFC_OTA_CHUNK_SIZE_ADDR);
  uint32_t expectedCrc = read_int_tag(s_tag, NFC_OTA_CRC32_ADDR);

  // Validate chunk size
  if (chunkSize == 0 || chunkSize > NFC_OTA_MAX_CHUNK_SIZE)
  {
    Serial.print("NFC OTA: invalid chunk size: ");
    Serial.println(chunkSize);
    ota_abort("bad chunk size");
    return;
  }

  // Read chunk data from mailbox
  uint16_t actualLen = 0;
  if (!mailbox_read(s_chunkBuffer, chunkSize, &actualLen))
  {
    Serial.println("NFC OTA: mailbox read failed");
    ota_abort("mailbox read failed");
    return;
  }

  if (actualLen != chunkSize)
  {
    Serial.print("NFC OTA: chunk size mismatch, expected ");
    Serial.print(chunkSize);
    Serial.print(" got ");
    Serial.println(actualLen);
    ota_abort("chunk size mismatch");
    return;
  }

  // Verify CRC32
  uint32_t actualCrc = crc32_calc(s_chunkBuffer, chunkSize);
  if (actualCrc != expectedCrc)
  {
    Serial.print("NFC OTA: CRC mismatch. Expected 0x");
    Serial.print(expectedCrc, HEX);
    Serial.print(" got 0x");
    Serial.println(actualCrc, HEX);
    ota_abort("crc mismatch");
    return;
  }

#if defined(ESP32)
  // Write to flash
  size_t written = Update.write(s_chunkBuffer, chunkSize);
  if (written != chunkSize)
  {
    Serial.print("Update.write failed: ");
    Serial.println(Update.errorString());
    ota_abort("write failed");
    return;
  }
#endif

  s_received += chunkSize;
  s_expectedChunk++;

  Serial.print("NFC OTA chunk ");
  Serial.print(chunkNum);
  Serial.print(": ");
  Serial.print(s_received);
  Serial.print("/");
  Serial.println(s_totalSize);

  // Signal ready for next chunk
  write_status(NFC_OTA_STATUS_READY);
  clear_command();
}

static void handle_end()
{
  if (s_state != OTA_STATE_MAILBOX_MODE && s_state != OTA_STATE_PREPARING)
  {
    clear_command();
    return;
  }

  Serial.println("NFC OTA: CMD_END received");

  // Disable mailbox
  mailbox_disable();

  // Verify all data received
  if (s_received != s_totalSize)
  {
    Serial.print("NFC OTA: size mismatch. Expected ");
    Serial.print(s_totalSize);
    Serial.print(" received ");
    Serial.println(s_received);
    ota_abort("size mismatch");
    return;
  }

#if defined(ESP32)
  if (!Update.end(true))
  {
    Serial.print("Update.end failed: ");
    Serial.println(Update.errorString());
    ota_abort("end failed");
    return;
  }

  Serial.println("NFC OTA success. Rebooting...");
  
  s_state = OTA_STATE_DONE;
  
  // Signal done before reboot
  write_status(NFC_OTA_STATUS_DONE);
  clear_command();
  
  delay(200);
  ESP.restart();
#else
  ota_abort("not ESP32");
#endif
}

static void handle_abort_cmd()
{
  Serial.println("NFC OTA: CMD_ABORT received");

#if defined(ESP32)
  if (Update.isRunning())
    Update.abort();
#endif

  // Disable mailbox
  mailbox_disable();

  s_state = OTA_STATE_IDLE;
  s_totalSize = 0;
  s_received = 0;
  s_expectedChunk = 0;

  write_status(NFC_OTA_STATUS_IDLE);
  clear_command();
}

} // namespace

void nfc_ota_setup(SFE_ST25DV64KC *tag)
{
  s_tag = tag;
  
  // Ensure mailbox is disabled initially
  mailbox_disable();
  
  // Initialize status to idle
  write_status(NFC_OTA_STATUS_IDLE);
  clear_command();
  
  Serial.println("NFC OTA: initialized (hybrid EEPROM + Mailbox mode)");
}

bool nfc_ota_poll()
{
  if (s_tag == nullptr)
    return false;

  // Read command from EEPROM
  uint32_t cmd = read_int_tag(s_tag, NFC_OTA_CMD_ADDR);

  switch (cmd)
  {
  case NFC_OTA_CMD_NONE:
    // No command pending
    break;

  case NFC_OTA_CMD_START:
    handle_start();
    break;

  case NFC_OTA_CMD_DATA_START:
    handle_data_start();
    break;

  case NFC_OTA_CMD_DATA:
    handle_data();
    break;

  case NFC_OTA_CMD_END:
    handle_end();
    break;

  case NFC_OTA_CMD_ABORT:
    handle_abort_cmd();
    break;

  default:
    // Unknown command, ignore
    break;
  }

  return (s_state != OTA_STATE_IDLE);
}

bool nfc_ota_in_progress()
{
  return (s_state != OTA_STATE_IDLE);
}
