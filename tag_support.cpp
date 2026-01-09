#include <Arduino.h>
#include <Wire.h>

#include "SparkFun_ST25DV64KC_Arduino_Library.h"

#include "tag_support.h"

extern int GLOBAL_ERROR;

#define ST25DV_I2C_ADDR_DATA  0x53  // ST25DV user memory I2C address

// RAW I2C read - bypasses SparkFun library completely
// Use this to test if the issue is library caching vs chip behavior
static bool raw_i2c_read(uint16_t address, uint8_t* data, uint8_t len)
{
  Wire.beginTransmission(ST25DV_I2C_ADDR_DATA);
  Wire.write((address >> 8) & 0xFF);  // MSB
  Wire.write(address & 0xFF);         // LSB
  if (Wire.endTransmission(false) != 0) {  // Repeated start
    return false;
  }
  
  uint8_t received = Wire.requestFrom(ST25DV_I2C_ADDR_DATA, len);
  if (received != len) {
    return false;
  }
  
  for (uint8_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }
  return true;
}

// Debug function: compare library read vs raw I2C read
void debug_compare_reads(SFE_ST25DV64KC* tag, int address)
{
  uint8_t lib_data[4], raw_data[4];
  
  // Library read
  tag->readEEPROM(address, lib_data, 4);
  
  // Raw I2C read
  raw_i2c_read(address, raw_data, 4);
  
  Serial.print(F("Address 0x"));
  Serial.print(address, HEX);
  Serial.print(F(" - Library: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(lib_data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(F(" | Raw I2C: "));
  for (int i = 0; i < 4; i++) {
    Serial.print(raw_data[i], HEX);
    Serial.print(" ");
  }
  
  if (memcmp(lib_data, raw_data, 4) != 0) {
    Serial.println(F(" *** MISMATCH! ***"));
  } else {
    Serial.println(F(" (match)"));
  }
}

// Force I2C cache invalidation on every read
// The RF_WRITE flag doesn't reliably toggle, so we always refresh
static void force_i2c_cache_refresh(SFE_ST25DV64KC* tag)
{
  // Temporarily disable RF to force internal sync
  tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, 0x2003, 0x02); // RF_DIS=1
  delayMicroseconds(500);
  tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, 0x2003, 0x00); // RF_DIS=0
  delayMicroseconds(500);
}

uint32_t read_int_tag(SFE_ST25DV64KC* tag, int address)
{
  uint32_t result = 0;
  uint8_t tagRead[4];
  if((address % 4) == 0){
    // Force cache refresh before every read
    force_i2c_cache_refresh(tag);
    
    tag->readEEPROM(address, tagRead, 4);
    Serial.print(tagRead[0], HEX);
    Serial.print(" ");
    Serial.print(tagRead[1], HEX);  
    Serial.print(" ");
    Serial.print(tagRead[2], HEX);
    Serial.print(" ");
    Serial.print(tagRead[3], HEX);
    Serial.print("--> ");

    for(uint8_t i =0; i<4; i++){
      result += ((tagRead[i]) << (8 * i));
    }
    return result;
  }
  GLOBAL_ERROR = 1;
  return 0;
}

void write_int_tag(SFE_ST25DV64KC* tag, int address, uint32_t value)
{
  //unprotect?
  uint32_t result = 0;
  uint8_t tagWrite[4];
  uint32_t tempvalue = value;
  if((address % 4) == 0){
    for(uint8_t i =0; i<4; i++){
      tagWrite[i] = ((value) >> i*8) & 0xFF;
    }
    tag->writeEEPROM(address, tagWrite, 4);
    return;
  }
  GLOBAL_ERROR = 1;
}

void write_string_tag(SFE_ST25DV64KC* tag, int address, uint8_t * stringtowrite, uint8_t string_len)
{
  //unprotect?
  if((address % 4) == 0){
    tag->writeEEPROM(address, stringtowrite, string_len);
    return;
  }
  GLOBAL_ERROR = 1;
}

void read_string_tag(SFE_ST25DV64KC* tag, int address, uint8_t * stringtoread, uint8_t string_len)
{
  //unprotect?
  if((address % 4) == 0){
    // Force cache refresh before every read
    force_i2c_cache_refresh(tag);
    
    tag->readEEPROM(address, stringtoread, string_len);
    return;
  }
  GLOBAL_ERROR = 1;
}


void setup_tag(SFE_ST25DV64KC* tag){
  if (!tag->begin(Wire))
  {
    Serial.println(F("ST25 not detected. Freezing..."));
    return;
  }

  Serial.println(F("ST25 connected."));

  // -=-=-=-=-=-=-=-=-

  Serial.println(F("Opening I2C security session with default password (all zeros)."));
  uint8_t password[8] = {0x0}; // Default password is all zeros
  tag->openI2CSession(password);

  Serial.print(F("I2C session is "));
  Serial.println(tag->isI2CSessionOpen() ? "opened." : "closed.");

  // Ensure the tag starts in non-mailbox mode.
  // (OTA-over-NFC can still enable mailbox later when needed.)
  {
    bool ok = true;

    // Disable mailbox enable (dynamic).
    ok &= tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::DATA, REG_MB_CTRL_DYN, 0x00);

    // Disable mailbox mode in FTM register (system).
    uint8_t ftm = 0;
    if (tag->st25_io.readSingleByte(SF_ST25DV64KC_ADDRESS::SYSTEM, REG_FTM, &ftm))
    {
      ftm = (uint8_t)(ftm & (uint8_t)~BIT_FTM_MB_MODE);
      ok &= tag->st25_io.writeSingleByte(SF_ST25DV64KC_ADDRESS::SYSTEM, REG_FTM, ftm);
    }
    else
    {
      ok = false;
    }

    if (!ok)
    {
      Serial.println(F("Warning: failed to disable mailbox mode"));
    }
  }


}

void dump_tag_words64(SFE_ST25DV64KC* tag, int startAddress)
{
  if (tag == nullptr)
  {
    Serial.println(F("dump_tag_words64: tag=null"));
    return;
  }
  if ((startAddress % 4) != 0)
  {
    Serial.println(F("dump_tag_words64: startAddress not 4-byte aligned"));
    return;
  }

  Serial.print(F("NFC EEPROM dump: 64 words from 0x"));
  Serial.println(startAddress, HEX);

  for (int i = 0; i < 64; i++)
  {
    int addr = startAddress + (i * 4);
    uint32_t v = read_int_tag(tag, addr);

    // Format: 0x0000: DEADBEEF
    Serial.print(addr);
    Serial.print(": ");
    // zero-pad to 8 hex digits
    for(int j = 0; j < 4; j++){
      Serial.print((v >> (8*j)) & 0xFF, HEX);
      Serial.print(" ");
      }
    Serial.println("");
  }
}