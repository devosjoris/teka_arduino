
#include <Arduino.h>
#include "senslog_fs.h"
#include <FS.h>
#include <LittleFS.h>


// Returns true if the ring entry exists.
bool nvs_read_entry(uint16_t index, uint32_t *sensorValue, uint32_t *unixTimestamp, bool *rtcValid)
{
  return senslog_read_entry(index, sensorValue, unixTimestamp, rtcValid);
}

void nvs_fix_invalid_timestamps(uint32_t rtc_old, uint32_t rtc_new)
{
  const uint16_t fixedCount = senslog_fix_invalid_timestamps(rtc_old, rtc_new);

  Serial.print("NVS timestamp fix: rtc_old=");
  Serial.print(rtc_old);
  Serial.print(" rtc_new=");
  Serial.print(rtc_new);
  Serial.print(" fixed=");
  Serial.println(fixedCount);
}

void nvs_log_packed(uint32_t sensorValue, uint32_t unixTimestamp)
{
  // // Flash wear guard: write at most once per minute
  // int64_t now_us = esp_timer_get_time();
  // if (last_nvs_write_us != 0 && (now_us - last_nvs_write_us) < (60LL * 1000LL * 1000LL)) {
  //   return;
  // }

  if (!senslog_log_packed(sensorValue, unixTimestamp, unixTimestamp > valid_time_threshold)) // valid_rtc_time is not available here, pass true or refactor as needed
    return;

  //read back the value to make sure it is ok:
  Serial.print("NVS log index: ");
  Serial.print((senslog_get_index()));

  Serial.print(" t = ");
  Serial.print(unixTimestamp);
  Serial.print(" value = ");
  Serial.print(sensorValue);
  Serial.print(" rtc_valid: ");
  Serial.print(unixTimestamp > valid_time_threshold ? "true" : "false");
  Serial.println();
}

// Dump all valid NVS log entries to Serial
void nvs_print_all_entries()
{
  Serial.println("=== NVS Log Dump ===");
  Serial.println("Index\tTimestamp\tValue\tRTC Valid\tDate");

  uint16_t count = 0;
  const uint16_t maxEntries = 10 * 24 * 60; // kRingSize from senslog_fs.cpp

  for (uint16_t i = 0; i < maxEntries; i++) {
    uint32_t sensorValue = 0;
    uint32_t unixTimestamp = 0;
    bool rtcValid = false;

    if (nvs_read_entry(i, &sensorValue, &unixTimestamp, &rtcValid)) {
      char dateStr[20];
      // You may need to move formatEpochSeconds to a shared location if not available here
      formatEpochSeconds(unixTimestamp, dateStr, sizeof(dateStr), false);

      Serial.print(i);
      Serial.print("\t");
      Serial.print(unixTimestamp);
      Serial.print("\t");
      Serial.print(sensorValue);
      Serial.print("\t");
      Serial.print(rtcValid? "valid":"invalid");
      Serial.print("\t");
      Serial.println(dateStr);
      count++;
    }
  }

  Serial.print("=== Total entries: ");
  Serial.print(count);
  Serial.println(" ===");
}

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

bool senslog_read_entry(uint16_t index, uint32_t *sensorValue, uint32_t *unixTimestamp, bool *rtcValid) {
  if (sensorValue)
    *sensorValue = 0;
  if (unixTimestamp)
    *unixTimestamp = 0;
  if (rtcValid)
    *rtcValid = false;
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
  
  if (rtcValid)
    *rtcValid= e.rtcValid ? true : false;

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

// Formats: "YYYY-MM-DD HH:MM:SS"
void formatEpochSeconds(uint32_t epochSec, char* out, size_t outSize, bool useLocal) {
  time_t t = (time_t)epochSec;
  struct tm tmval;
#if defined(ESP32) || defined(ESP8266)
  // ESP platforms have localtime_r/gmtime_r
  if (useLocal) localtime_r(&t, &tmval);
  else gmtime_r(&t, &tmval);
#else
  // Fallback: non-thread-safe, but fine on Arduino
  struct tm* p = useLocal ? localtime(&t) : gmtime(&t);
  tmval = *p;
#endif
  // snprintf(out, outSize, "%04d-%02d-%02d %02d:%02d:%02d",
  //          tmval.tm_year + 1900, tmval.tm_mon + 1, tmval.tm_mday,
  //          tmval.tm_hour, tmval.tm_min, tmval.tm_sec);

  snprintf(out, outSize, "%02d-%02d-%04d",
           tmval.tm_mday, tmval.tm_mon + 1, tmval.tm_year + 1900);
}

uint16_t senslog_fix_invalid_timestamps(uint32_t rtc_old, uint32_t rtc_new) {
  Serial.println("=== NVS TS FIXING ===");
  Serial.println("Index\tTimestamp\tValue\tDate");

  uint16_t count = 0;
  const uint16_t maxEntries = 10 * 24 * 60; // kRingSize from senslog_fs.cpp

  for (uint16_t i = 0; i < maxEntries; i++) {
    uint32_t sensorValue = 0;
    uint32_t unixTimestamp = 0;
    bool rtcValid = false;

    if (nvs_read_entry(i, &sensorValue, &unixTimestamp, &rtcValid)) {
      if(rtcValid == false) {
         ;agj;ad;das  TODO FIX THIS
      }

      count++;
    }
  }

  Serial.print("=== Total entries: ");
  Serial.print(count);
  Serial.println(" ===");
}

uint16_t senslog_get_magic(void) {
  if (!senslog_init())
    return 0;
  return g_meta.magic;
}

uint16_t senslog_get_index(void) {
  if (!senslog_init())
    return 0;
  return g_meta.idx;
}
