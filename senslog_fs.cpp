#include "senslog_fs.h"

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

namespace {

static constexpr uint16_t kMagic = 0x501E; // bump to force re-init when format changes
static constexpr uint16_t kRingSize = 10 * 24 * 60;

static const char *kMetaPath = "/senslog.meta";
static const char *kRingPath = "/senslog.bin";

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint16_t idx;
  uint32_t last;
  uint32_t tlast;
} LogMeta;

typedef struct __attribute__((packed)) {
  uint32_t sensorValue;
  uint32_t unixTimestamp;
  uint8_t rtcValid;
  uint8_t _pad[3];
} LogEntry;

static LogMeta g_meta = {0};
static bool g_ready = false;

static bool write_meta(const LogMeta &m) {
  File f = LittleFS.open(kMetaPath, "w");
  if (!f)
    return false;
  const size_t w = f.write((const uint8_t *)&m, sizeof(m));
  f.close();
  return w == sizeof(m);
}

static bool read_meta(LogMeta *out) {
  if (!out)
    return false;

  File f = LittleFS.open(kMetaPath, "r");
  if (!f)
    return false;
  if ((size_t)f.size() != sizeof(LogMeta)) {
    f.close();
    return false;
  }
  const size_t n = f.read((uint8_t *)out, sizeof(LogMeta));
  f.close();
  return n == sizeof(LogMeta);
}

static bool ensure_ring_file() {
  const size_t expectedSize = (size_t)kRingSize * sizeof(LogEntry);

  if (LittleFS.exists(kRingPath)) {
    File f = LittleFS.open(kRingPath, "r");
    if (!f)
      return false;
    const size_t actualSize = (size_t)f.size();
    f.close();
    if (actualSize == expectedSize)
      return true;

    Serial.println("LittleFS ring size mismatch; recreating");
    LittleFS.remove(kRingPath);
  }

  File f = LittleFS.open(kRingPath, "w");
  if (!f)
    return false;

  static uint8_t zeros[256] = {0};
  size_t remaining = expectedSize;
  while (remaining > 0) {
    const size_t chunk = (remaining > sizeof(zeros)) ? sizeof(zeros) : remaining;
    if (f.write(zeros, chunk) != chunk) {
      f.close();
      return false;
    }
    remaining -= chunk;
  }
  f.close();
  return true;
}

} // namespace

bool senslog_init(void) {
  if (g_ready)
    return true;

  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed; formatting...");
    if (!LittleFS.begin(true)) {
      Serial.println("LittleFS mount failed after format");
      return false;
    }
  }

  LogMeta m;
  const bool haveMeta = read_meta(&m);
  if (!haveMeta || m.magic != kMagic) {
    Serial.println("LOG init/format");
    m.magic = kMagic;
    m.idx = 0;
    m.last = 0;
    m.tlast = 0;
    if (!write_meta(m))
      return false;
    LittleFS.remove(kRingPath);
  }

  if (!ensure_ring_file())
    return false;

  if (!read_meta(&g_meta))
    return false;

  g_ready = true;
  return true;
}

bool senslog_read_entry(uint16_t index, uint32_t *sensorValue, uint32_t *unixTimestamp) {
  if (sensorValue)
    *sensorValue = 0;
  if (unixTimestamp)
    *unixTimestamp = 0;

  if (!senslog_init())
    return false;

  const uint16_t slot = (uint16_t)(index % kRingSize);
  const size_t offsetBytes = (size_t)slot * sizeof(LogEntry);

  File f = LittleFS.open(kRingPath, "r");
  if (!f)
    return false;
  if (!f.seek(offsetBytes, SeekSet)) {
    f.close();
    return false;
  }

  LogEntry e;
  const size_t n = f.read((uint8_t *)&e, sizeof(e));
  f.close();
  if (n != sizeof(e))
    return false;

  // Unwritten slots are all zeros.
  if (e.unixTimestamp == 0 && e.sensorValue == 0)
    return false;

  if (sensorValue)
    *sensorValue = e.sensorValue;
  if (unixTimestamp)
    *unixTimestamp = e.unixTimestamp;
  return true;
}

bool senslog_log_packed(uint32_t sensorValue, uint32_t unixTimestamp, bool rtcValid) {
  if (!senslog_init())
    return false;

  const uint16_t idx = g_meta.idx;
  const uint16_t slot = (uint16_t)(idx % kRingSize);
  const size_t offsetBytes = (size_t)slot * sizeof(LogEntry);

  File f = LittleFS.open(kRingPath, "r+");
  if (!f)
    return false;
  if (!f.seek(offsetBytes, SeekSet)) {
    f.close();
    return false;
  }

  LogEntry e;
  e.sensorValue = sensorValue;
  e.unixTimestamp = unixTimestamp;
  e.rtcValid = rtcValid ? 1 : 0;
  e._pad[0] = 0;
  e._pad[1] = 0;
  e._pad[2] = 0;

  const size_t w = f.write((const uint8_t *)&e, sizeof(e));
  f.close();
  if (w != sizeof(e))
    return false;

  g_meta.last = sensorValue;
  g_meta.tlast = unixTimestamp;
  g_meta.idx = (uint16_t)((idx + 1) % kRingSize);
  (void)write_meta(g_meta);

  return true;
}

uint16_t senslog_fix_invalid_timestamps(uint32_t rtc_old, uint32_t rtc_new) {
  if (!senslog_init())
    return 0;

  const int64_t offset = (int64_t)rtc_new - (int64_t)rtc_old;
  uint16_t fixedCount = 0;

  File f = LittleFS.open(kRingPath, "r+");
  if (!f)
    return 0;

  for (uint16_t i = 0; i < kRingSize; i++) {
    const size_t offsetBytes = (size_t)i * sizeof(LogEntry);
    if (!f.seek(offsetBytes, SeekSet))
      continue;

    LogEntry e;
    const size_t n = f.read((uint8_t *)&e, sizeof(e));
    if (n != sizeof(e))
      continue;

    if (e.unixTimestamp == 0 && e.sensorValue == 0)
      continue;

    if (e.rtcValid)
      continue;

    const uint32_t rtc_invalid = e.unixTimestamp;
    const int64_t fixed64 = (int64_t)rtc_invalid + offset;
    if (fixed64 < 0 || fixed64 > 0xFFFFFFFFLL)
      continue;

    e.unixTimestamp = (uint32_t)fixed64;
    e.rtcValid = 1;

    if (!f.seek(offsetBytes, SeekSet))
      continue;
    const size_t w = f.write((const uint8_t *)&e, sizeof(e));
    if (w != sizeof(e))
      continue;

    fixedCount++;
  }

  f.close();
  return fixedCount;
}

uint16_t senslog_get_magic(void) {
  if (!senslog_init())
    return 0;
  return g_meta.magic;
}
