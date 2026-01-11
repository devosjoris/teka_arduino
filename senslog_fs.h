
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define valid_time_threshold (1767370414u) //1 jan 2026

// Sensor log storage backed by LittleFS.
// - A fixed-size ring buffer lives in /senslog.bin
// - Small metadata (magic/idx/last/tlast) lives in /senslog.meta

#ifdef __cplusplus
extern "C" {
#endif



// API for NVS log operations
bool nvs_read_entry(uint16_t index, uint32_t *sensorValue, uint32_t *unixTimestamp, bool *rtcValid);
void nvs_fix_invalid_timestamps(uint32_t rtc_old, uint32_t rtc_new);
void nvs_log_packed(uint32_t sensorValue, uint32_t unixTimestamp);
void nvs_print_all_entries();

bool senslog_init(void);

// Returns true if a valid entry exists at index.
bool senslog_read_entry(uint16_t index, uint32_t *sensorValue, uint32_t *unixTimestamp, bool *rtcValid);

// Writes one entry at the current ring index; advances index on success.
// Returns true if the write succeeded.
bool senslog_log_packed(uint32_t sensorValue, uint32_t unixTimestamp, bool rtcValid);

// Updates an existing entry at a specific index (does not advance ring index).
// Returns true if the write succeeded.
bool senslog_update_entry(uint16_t index, uint32_t sensorValue, uint32_t unixTimestamp, bool rtcValid);

// Applies RTC offset fix to entries that were logged while rtcValid==false.
// Returns the number of entries fixed.
uint16_t senslog_fix_invalid_timestamps(uint32_t rtc_old, uint32_t rtc_new);

uint16_t senslog_get_magic(void);
uint16_t senslog_get_index(void);

void formatEpochSeconds(uint32_t epochSec, char* out, size_t outSize, bool useLocal = false);

#ifdef __cplusplus
} // extern "C"
#endif
