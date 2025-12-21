#include "nfc_ota.h"

#include <Wire.h>

#include "SparkFun_ST25DV64KC_Arduino_Library.h"

#if defined(ESP32)
#include <Update.h>
#endif

namespace {

static SFE_ST25DV64KC *s_tag = nullptr;

static bool s_inProgress = false;
static uint32_t s_totalSize = 0;
static uint32_t s_received = 0;
static uint16_t s_expectedSeq = 0;

// CRC32 (Ethernet, reversed polynomial)
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
  crc = ~crc;
  while (len--)
  {
    crc ^= *data++;
    for (int i = 0; i < 8; i++)
      crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
  }
  return ~crc;
}

static uint32_t s_crc32 = 0;

// Mailbox message format (little-endian):
//   u32 magic = 'N''F''O''T' (0x544F464E)
//   u8  version = 1
//   u8  type: 1=START, 2=DATA, 3=END, 4=ABORT
//   u16 seq
//   u32 arg0: START=totalSize, DATA=offset, END=expectedCrc32
//   u16 dataLen
//   u16 reserved
//   data[dataLen]
static constexpr uint32_t kMagic = 0x544F464Eu;
static constexpr uint8_t kVersion = 1;

enum MsgType : uint8_t
{
  MSG_START = 1,
  MSG_DATA = 2,
  MSG_END = 3,
  MSG_ABORT = 4,
};

#pragma pack(push, 1)
struct MsgHeader
{
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t seq;
  uint32_t arg0;
  uint16_t dataLen;
  uint16_t reserved;
};
#pragma pack(pop)

static bool mailbox_enable()
{
  if (s_tag == nullptr)
    return false;

  // Enable mailbox mode in FTM register and enable mailbox itself.
  bool ok = true;
  ok &= s_tag->st25_io.setRegisterBit(SF_ST25DV64KC_ADDRESS::SYSTEM, REG_FTM, BIT_FTM_MB_MODE);
  ok &= s_tag->st25_io.setRegisterBit(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, BIT_MB_CTRL_DYN_MB_EN);
  return ok;
}

static bool mailbox_read(uint8_t *out, uint16_t outCap, uint16_t *outLen)
{
  *outLen = 0;
  if (s_tag == nullptr)
    return false;

  uint8_t ctrl = 0;
  if (!s_tag->st25_io.readSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, &ctrl))
    return false;

  if ((ctrl & BIT_MB_CTRL_DYN_MB_EN) == 0)
  {
    if (!mailbox_enable())
      return false;
    // Re-read control after enabling
    if (!s_tag->st25_io.readSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, &ctrl))
      return false;
  }

  if ((ctrl & BIT_MB_CTRL_DYN_RF_PUT_MSG) == 0)
    return false; // no new RF message

  uint8_t len8 = 0;
  if (!s_tag->st25_io.readSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_LEN_DYN, &len8))
    return false;

  uint16_t len = len8;
  if (len == 0)
  {
    // Acknowledge/clear anyway.
    s_tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, BIT_MB_CTRL_DYN_MB_EN);
    return true;
  }

  // Mailbox max is 255, but if something reports an unexpected length, clear and ignore.
  if (len > outCap)
  {
    s_tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, BIT_MB_CTRL_DYN_MB_EN);
    return true;
  }

  if (!s_tag->st25_io.readMultipleBytes(SF_ST25DV64KC_ADDRESS::DATA, MAILBOX_BASE, out, len))
    return false;

  *outLen = len;

  // Best-effort acknowledge/clear RF message.
  // (On ST25DV, RF flags are dynamic; writing MB_EN-only typically releases the message.)
  s_tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, BIT_MB_CTRL_DYN_MB_EN);

  return true;
}

static void ota_abort(const char *reason)
{
  Serial.print("NFC OTA abort: ");
  Serial.println(reason);

#if defined(ESP32)
  if (Update.isRunning())
    Update.abort();
#endif

  s_inProgress = false;
  s_totalSize = 0;
  s_received = 0;
  s_expectedSeq = 0;
  s_crc32 = 0;
}

static void handle_start(const MsgHeader &h)
{
  if (h.arg0 == 0)
  {
    ota_abort("size=0");
    return;
  }

  Serial.print("NFC OTA start, size=");
  Serial.println(h.arg0);

#if defined(ESP32)
  if (!Update.begin(h.arg0, U_FLASH))
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

  s_inProgress = true;
  s_totalSize = h.arg0;
  s_received = 0;
  s_expectedSeq = h.seq + 1;
  s_crc32 = 0;
}

static void handle_data(const MsgHeader &h, const uint8_t *payload, uint16_t payloadLen)
{
  if (!s_inProgress)
    return;

  if (h.seq != s_expectedSeq)
  {
    ota_abort("bad seq");
    return;
  }

  if (h.arg0 != s_received)
  {
    ota_abort("bad offset");
    return;
  }

  if (payloadLen != h.dataLen)
  {
    ota_abort("bad length");
    return;
  }

  if (payloadLen == 0)
  {
    ota_abort("empty chunk");
    return;
  }

#if defined(ESP32)
  size_t written = Update.write((uint8_t *)payload, payloadLen);
  if (written != payloadLen)
  {
    Serial.print("Update.write failed: ");
    Serial.println(Update.errorString());
    ota_abort("write failed");
    return;
  }
#endif

  s_crc32 = crc32_update(s_crc32, payload, payloadLen);
  s_received += payloadLen;
  s_expectedSeq++;

  Serial.print("NFC OTA chunk: ");
  Serial.print(s_received);
  Serial.print("/");
  Serial.println(s_totalSize);
}

static void handle_end(const MsgHeader &h)
{
  if (!s_inProgress)
    return;

  if (h.seq != s_expectedSeq)
  {
    ota_abort("bad seq end");
    return;
  }

  if (s_received != s_totalSize)
  {
    ota_abort("size mismatch");
    return;
  }

  uint32_t expectedCrc = h.arg0;
  if (expectedCrc != 0 && expectedCrc != s_crc32)
  {
    Serial.print("CRC mismatch. Expected ");
    Serial.print(expectedCrc, HEX);
    Serial.print(" got ");
    Serial.println(s_crc32, HEX);
    ota_abort("crc mismatch");
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
  delay(200);
  ESP.restart();
#else
  ota_abort("not ESP32");
#endif
}

static void handle_packet(const uint8_t *buf, uint16_t len)
{
  if (len < sizeof(MsgHeader))
    return;

  MsgHeader h;
  memcpy(&h, buf, sizeof(h));

  if (h.magic != kMagic || h.version != kVersion)
    return;

  if ((sizeof(MsgHeader) + h.dataLen) > len)
    return;

  const uint8_t *payload = buf + sizeof(MsgHeader);

  switch (h.type)
  {
  case MSG_START:
    handle_start(h);
    break;
  case MSG_DATA:
    handle_data(h, payload, h.dataLen);
    break;
  case MSG_END:
    handle_end(h);
    break;
  case MSG_ABORT:
    ota_abort("remote abort");
    break;
  default:
    break;
  }
}

} // namespace

void nfc_ota_setup(SFE_ST25DV64KC *tag)
{
  s_tag = tag;
  if (!mailbox_enable())
  {
    Serial.println("NFC OTA: failed to enable mailbox");
    return;
  }
  Serial.println("NFC OTA: mailbox enabled");
}

bool nfc_ota_poll()
{
  if (s_tag == nullptr)
    return false;

  // Mailbox max length is 255.
  static uint8_t mbuf[255];
  uint16_t mlen = 0;

  bool got = mailbox_read(mbuf, sizeof(mbuf), &mlen);
  if (!got)
    return s_inProgress;

  if (mlen > 0)
    handle_packet(mbuf, mlen);

  return s_inProgress;
}

bool nfc_ota_in_progress()
{
  return s_inProgress;
}
