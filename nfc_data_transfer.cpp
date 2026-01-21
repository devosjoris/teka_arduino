#include "nfc_data_transfer.h"
#include "tag_support.h"
#include "senslog_fs.h"
#include "pinmap.h"

#include "SparkFun_ST25DV64KC_Arduino_Library.h"

// External function from main sketch to sync RTC from NFC tag
extern void sync_rtc_from_tag(void);

namespace {

static SFE_ST25DV64KC* s_tag = nullptr;
static bool s_initialized = false;

// Indices of entries sent in the last batch (for marking on ACK)
static uint16_t s_lastBatchIndices[NFC_DT_MAX_ENTRIES];
static uint16_t s_lastBatchCount = 0;

// Last RTC sync time (ms since boot)
static uint32_t s_lastRtcSyncMs = 0;
static const uint32_t RTC_SYNC_INTERVAL_MS = 120000; // 2 minutes

// CRC32 (Ethernet, reversed polynomial) - same as nfc_ota.cpp
static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len)
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

// Write status to tag
static void write_status(uint32_t status)
{
    write_int_tag(s_tag, NFC_DT_STATUS_ADDR, status);
}

// Read command from tag
static uint32_t read_command(void)
{
    return read_int_tag(s_tag, NFC_DT_CMD_ADDR);
}

// Clear command field
static void clear_command(void)
{
    write_int_tag(s_tag, NFC_DT_CMD_ADDR, NFC_DT_CMD_NONE);
}

// Count entries that don't have READOUT_DONE flag set
static uint16_t count_pending_entries(void)
{
    const uint16_t ringSize = senslog_get_ring_size();
    uint16_t count = 0;
    
    void* fh = senslog_open_ring_read();
    if (!fh)
        return 0;
    
    for (uint16_t i = 0; i < ringSize; i++)
    {
        uint32_t sensorValue = 0;
        uint32_t timestamp = 0;
        uint8_t flags = 0;
        
        if (senslog_read_entry_raw_from_file(fh, i, &sensorValue, &timestamp, &flags))
        {
            // Entry exists (non-zero) and not yet read out
            if ((flags & SENSLOG_FLAG_READOUT_DONE) == 0)
            {
                count++;
            }
        }
    }
    
    senslog_close_ring_file(fh);
    
    return count;
}

// Handle REQUEST_DATA command
static void handle_request_data(void)
{
    Serial.println(F("NFC_DT: Processing data request"));
    
    write_status(NFC_DT_STATUS_BUSY);

    // Only sync RTC if more than 1 minute since last sync
    uint32_t now = millis();
    if (s_lastRtcSyncMs == 0 || (now - s_lastRtcSyncMs) >= RTC_SYNC_INTERVAL_MS)
    {
        sync_rtc_from_tag();
        s_lastRtcSyncMs = now;
    }
    
    const uint16_t ringSize = senslog_get_ring_size();
    
    // Scan for entries without READOUT_DONE flag
    s_lastBatchCount = 0;
    uint32_t crc = 0;
    uint16_t addr = NFC_DT_DATA_START_ADDR;
    uint16_t totalPending = 0;
    
    void* fh = senslog_open_ring_read();
    if (!fh)
    {
        Serial.println(F("NFC_DT: Failed to open ring file"));
        write_status(NFC_DT_STATUS_ERROR);
        clear_command();
        return;
    }
    
    Serial.println(F("NFC_DT: write to eeprom:"));

    //we loop over all entries, count the pending ones
    //then after the loop we write the first NFC_DT_MAX_ENTRIES to eeprom and wait for a new poll -> ack/nack from device
    //not very efficient because we read all entries every time (but flash read is fast anyhow so this should work...)
    for (uint16_t i = 0; i < ringSize; i++) 
    {
        uint32_t sensorValue = 0;
        uint32_t timestamp = 0;
        uint8_t flags = 0;
        
        if (senslog_read_entry_raw_from_file(fh, i, &sensorValue, &timestamp, &flags))
        {
            // Entry exists - check if already read out
            if ((flags & SENSLOG_FLAG_READOUT_DONE) == 0)
            {
                totalPending++;
                
                if (s_lastBatchCount < NFC_DT_MAX_ENTRIES)
                {
                    //Serial.println(i);
                    // Remember this index for marking later
                    s_lastBatchIndices[s_lastBatchCount] = i;
                    s_lastBatchCount++;
                    
                    // Build entry for NFC
                    NfcDtLogEntry entry;
                    entry.sensorValue = sensorValue;
                    entry.unixTimestamp = timestamp;
                    entry.flags = flags;
                    entry._pad[0] = 0;
                    entry._pad[1] = 0;
                    entry._pad[2] = 0;
                    
                    // Update CRC
                    crc = crc32_update(crc, (const uint8_t*)&entry, sizeof(entry));
                    
                    // Write to tag
                    write_string_tag(s_tag, addr, (uint8_t*)&entry, sizeof(entry));
                    //check if write was successful
                    int readback_sensorValue = read_int_tag(s_tag, addr);
                    int readback_unixTimestamp = read_int_tag(s_tag, addr+4);
                    int readback_flags = read_int_tag(s_tag, addr+8);//padded with zeros
                    //write ok?
                    if(readback_sensorValue != sensorValue || readback_unixTimestamp != timestamp || readback_flags != flags){
                        Serial.println(F("NFC_DT: Write verification failed"));
                    }
                    addr += sizeof(entry);
                }
                //when totalpending is larger than max entries
                //we can stop, the app wil know that total pending > max entries so it will read again

                if(totalPending > NFC_DT_MAX_ENTRIES){
                    Serial.println(F("NFC_DT: More pending entries than max batch size"));
                    i = ringSize; //break the loop
                }
            }
        }
    }
    
    senslog_close_ring_file(fh);
    
    if (s_lastBatchCount == 0)
    {
        Serial.println(F("NFC_DT: No new data"));
        write_int_tag(s_tag, NFC_DT_FIELD_COUNT_ADDR, 0);
        write_int_tag(s_tag, NFC_DT_TOTAL_PENDING_ADDR, 0);
        write_status(NFC_DT_STATUS_NO_DATA);
        clear_command();
        return;
    }
    
    const uint16_t dataLen = s_lastBatchCount * NFC_DT_ENTRY_SIZE;
    
    Serial.print(F("NFC_DT: Sending "));
    Serial.print(s_lastBatchCount);
    Serial.print(F(" entries, "));
    Serial.print(totalPending);
    Serial.println(F(" total pending"));
    
    // Write metadata
    write_int_tag(s_tag, NFC_DT_FIELD_COUNT_ADDR, s_lastBatchCount);
    write_int_tag(s_tag, NFC_DT_TOTAL_PENDING_ADDR, totalPending);
    write_int_tag(s_tag, NFC_DT_DATA_LEN_ADDR, dataLen);
    write_int_tag(s_tag, NFC_DT_CRC32_ADDR, crc);
    
    Serial.print(F("NFC_DT: CRC32 = 0x"));
    Serial.println(crc, HEX);
    
    write_status(NFC_DT_STATUS_DATA_READY);
    Serial.println(F("NFC_DT: Data ready"));
    clear_command();
}

// Handle ACK_DATA command
static void handle_ack_data(void)
{
    Serial.println(F("NFC_DT: Processing ACK"));
    
    // Mark all entries from the last batch as READOUT_DONE using batch write
    const uint16_t markedCount = senslog_set_flags_batch(s_lastBatchIndices, s_lastBatchCount, SENSLOG_FLAG_READOUT_DONE);
    
    Serial.print(F("NFC_DT: Marked "));
    Serial.print(markedCount);
    Serial.println(F(" entries as read"));
    
    s_lastBatchCount = 0;
    
    write_status(NFC_DT_STATUS_ACK_OK);
    clear_command();
}

// Handle NACK_DATA command
static void handle_nack_data(void)
{
    Serial.println(F("NFC_DT: NACK received, keeping batch for retry"));
    
    // Don't clear s_lastBatchIndices - app can request again
    write_status(NFC_DT_STATUS_IDLE);
    clear_command();
}

// Handle RESET_FLAGS command - clear all READOUT_DONE flags
static void handle_reset_flags(void)
{
    Serial.println(F("NFC_DT: Resetting all readout flags"));
    
    const uint16_t ringSize = senslog_get_ring_size();
    uint16_t clearedCount = 0;
    
    // First pass: collect indices that need clearing (with file open once for reading)
    static uint16_t indicesToClear[256];  // Reasonable batch size
    uint16_t numToClear = 0;
    
    void* fh = senslog_open_ring_read();
    if (fh)
    {
        for (uint16_t i = 0; i < ringSize && numToClear < 256; i++)
        {
            uint32_t sensorValue = 0;
            uint32_t timestamp = 0;
            uint8_t flags = 0;
            
            if (senslog_read_entry_raw_from_file(fh, i, &sensorValue, &timestamp, &flags))
            {
                if (flags & SENSLOG_FLAG_READOUT_DONE)
                {
                    indicesToClear[numToClear++] = i;
                }
            }
        }
        senslog_close_ring_file(fh);
    }
    
    // Second pass: clear flags using batch write for better performance
    clearedCount = senslog_clear_flags_batch(indicesToClear, numToClear, SENSLOG_FLAG_READOUT_DONE);
    
    Serial.print(F("NFC_DT: Cleared flags on "));
    Serial.print(clearedCount);
    Serial.println(F(" entries"));
    
    s_lastBatchCount = 0;
    
    write_status(NFC_DT_STATUS_ACK_OK);
    clear_command();
}

} // namespace

void nfc_dt_init(SFE_ST25DV64KC* tag)
{
    if (tag == nullptr)
    {
        Serial.println(F("NFC_DT: Init failed - tag is null"));
        return;
    }
    
    s_tag = tag;
    s_lastBatchCount = 0;
    
    // Initialize status
    write_status(NFC_DT_STATUS_IDLE);
    clear_command();
    
    s_initialized = true;
    
    Serial.println(F("NFC_DT: Initialized (flag-based tracking)"));
}

bool nfc_dt_poll(void)
{
    Serial.print("p");
    if (!s_initialized || s_tag == nullptr)
    {
        return false;
    }
    
    const uint32_t cmd = read_command();
    
    switch (cmd)
    {
    case NFC_DT_CMD_NONE:
        return false;
        
    case NFC_DT_CMD_REQUEST_DATA:
        handle_request_data();
        return true;
        
    case NFC_DT_CMD_ACK_DATA:
        handle_ack_data();
        return true;
        
    case NFC_DT_CMD_NACK_DATA:
        handle_nack_data();
        return true;
        
    case NFC_DT_CMD_RESET_FLAGS:
        handle_reset_flags();
        return true;
        
    default:
        Serial.print(F("NFC_DT: Unknown command: 0x"));
        Serial.println(cmd, HEX);
        write_status(NFC_DT_STATUS_ERROR);
        clear_command();
        return true;
    }
}

uint16_t nfc_dt_get_pending_count(void)
{
    return count_pending_entries();
}

void nfc_dt_reset_all_flags(void)
{
    handle_reset_flags();
}
