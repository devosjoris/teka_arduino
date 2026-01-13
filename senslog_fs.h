
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define valid_time_threshold (1767370414u) //1 jan 2026

// Flags byte bit definitions
#define SENSLOG_FLAG_RTC_VALID    0x01  // Bit 0: RTC was valid when entry was logged
#define SENSLOG_FLAG_READOUT_DONE 0x02  // Bit 1: Entry has been read out via NFC

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
uint16_t senslog_get_ring_size(void);

// Read entry with raw flags byte (for NFC transfer)
bool senslog_read_entry_raw(uint16_t index, uint32_t *sensorValue, uint32_t *unixTimestamp, uint8_t *flags);

// Set or clear specific flag bits on an entry
bool senslog_set_flags(uint16_t index, uint8_t flagsToSet);
bool senslog_clear_flags(uint16_t index, uint8_t flagsToClear);

void formatEpochSeconds(uint32_t epochSec, char* out, size_t outSize, bool useLocal = false);

#ifdef __cplusplus
} // extern "C"
#endif
