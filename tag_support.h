#ifndef TAG_SUPPORT_H
#define TAG_SUPPORT_H

#include <Arduino.h>

class SFE_ST25DV64KC;

uint32_t read_int_tag(SFE_ST25DV64KC* tag, int address);
void write_int_tag(SFE_ST25DV64KC* tag, int address, uint32_t value);
void write_string_tag(SFE_ST25DV64KC* tag, int address, uint8_t * stringtowrite, uint8_t string_len);
void read_string_tag(SFE_ST25DV64KC* tag, int address, uint8_t * stringtoread, uint8_t string_len);
void setup_tag(SFE_ST25DV64KC* tag);

#endif // TAG_SUPPORT_H
