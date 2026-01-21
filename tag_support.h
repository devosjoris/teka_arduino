#ifndef TAG_SUPPORT_H
#define TAG_SUPPORT_H

#include <Arduino.h>

class SFE_ST25DV64KC;
void disable_rf(SFE_ST25DV64KC* tag);
void enable_rf(SFE_ST25DV64KC* tag);

uint32_t read_int_tag(SFE_ST25DV64KC* tag, int address);
void write_int_tag(SFE_ST25DV64KC* tag, int address, uint32_t value);
void write_string_tag(SFE_ST25DV64KC* tag, int address, uint8_t * stringtowrite, uint8_t string_len);
void read_string_tag(SFE_ST25DV64KC* tag, int address, uint8_t * stringtoread, uint8_t string_len);
void setup_tag(SFE_ST25DV64KC* tag);

// Debug helper: dump 64 32-bit words (256 bytes) starting at startAddress.
// Each word is read using read_int_tag() so startAddress must be 4-byte aligned.
void dump_tag_words64(SFE_ST25DV64KC* tag, int startAddress = 0);

#endif // TAG_SUPPORT_H
