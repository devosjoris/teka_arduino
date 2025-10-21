#include <Wire.h>
#include "SparkFun_BMV080_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BMV080

SparkFunBMV080 bmv080; // Create an instance of the BMV080 class
#define BMV080_ADDR 0x57  // SparkFun BMV080 Breakout defaults to 0x57

// Some Dev boards have their QWIIC connector on Wire or Wire1
// This #ifdef will help this sketch work across more products

#include "SparkFun_ST25DV64KC_Arduino_Library.h" // Click here to get the library:  http://librarymanager/All#SparkFun_ST25DV64KC
SFE_ST25DV64KC tag;

//MEMORY MAP
// Setting options for:
// - Displayable name, approx. 25 - 30 characters (e.g. John Doe or XYZ Inc.);
// - Individual warning value [mg/m³] (cannot be set higher than the limit value);
// - individual limit value [mg/m³] (cannot be set lower than the warning value and maximum 5 mg/m³);
// - Operating mode (mode 1: measurement every minute; mode 2: measurement every 5 minutes);


//ALL 32 bit numbers

#define MEM_PTR_TIMESTAMP           0
#define MEM_VAL_TIMESTAMP           4
#define MEM_PTR_LAST_WRITE          8
#define MEM_PTR_LAST_READ           12

#define MEM_VAL_MEASURE_MODE        16
#define MEM_VAL_WARNING             20    // MAX MIN(5000, MEM_VAL_LIMIT) 
#define MEM_VAL_LIMIT               24    // MAX 7000

#define MEM_VAL_USER_NAME_LENGTH    44
#define MEM_VAL_USER_NAME           48

#define MEM_VAL_DATA_VALID          92

#define MEM_VAL_DATA_START          (100 *4)
#define MEM_VAL_DATA_END            (7500)

#define FW_REV                       1

int GLOBAL_ERROR =0;
uint16_t current_data_add     = MEM_VAL_DATA_START;
uint8_t measurement_mode      = 1;
uint8_t user_name_length      = 0;
uint8_t user_name[30];

uint16_t find_write_addr(uint16_t guess_addr){
  for(uint16_t i =0; i< (60000/8); i++){
    uint32_t temp_addr = guess_addr +i;
    if(temp_addr >= MEM_VAL_DATA_END){
      temp_addr = (temp_addr - MEM_VAL_DATA_END) + MEM_VAL_DATA_START;
    }
    if(read_int_tag(temp_addr) == 0xC1EAC1EA){
      if(i > 25){ //do not update it on each run to keep the number of writes low:::
        write_int_tag(MEM_PTR_LAST_WRITE, temp_addr);
      }
      return temp_addr;
    }
  }
  GLOBAL_ERROR = 1;
}

void init_memspace(){
  if(read_int_tag(MEM_VAL_DATA_VALID) != (((0x501d) << 16) + FW_REV)){ //not yet initialized
    write_int_tag(MEM_PTR_LAST_WRITE, MEM_VAL_DATA_START);
    write_int_tag(MEM_PTR_LAST_READ,  MEM_VAL_DATA_START);

    write_int_tag(MEM_VAL_DATA_START, 0xC1EAC1EA); //empty field -> ready to write:::
    
    write_string_tag(MEM_VAL_USER_NAME_LENGTH, 23);
    write_string_tag(MEM_VAL_USER_NAME, "USE APP TO SET USERNAME", 23);

    write_int_tag(MEM_VAL_MEASURE_MODE, 0x0001);     
    write_int_tag(MEM_VAL_WARNING, 5000);
    write_int_tag(MEM_VAL_LIMIT, 7000);

    write_int_tag(MEM_VAL_DATA_VALID, (((0x501d) << 16) + FW_REV));
  }
  current_data_add = read_int_tag(MEM_PTR_LAST_WRITE);
  current_data_add = find_write_addr(current_data_add);
  measurement_mode = read_int_tag(MEM_VAL_MEASURE_MODE);
  if(measurement_mode > 2) measurement_mode =1; //invalid value 
  user_name_length = read_int_tag(MEM_VAL_USER_NAME_LENGTH);
  read_string_tag(MEM_VAL_USER_NAME_LENGTH, user_name, user_name_length);
}

uint32_t read_int_tag(int address)
{
  uint32_t result = 0;
  uint8_t tagRead[4];
  if((address % 4) == 0){
    tag.readEEPROM(address, tagRead, 4);
    for(uint8_t i =0; i<4; i++){
      result += ((tagRead[i]) << (8 * i));
    }
    return result;
  }
  GLOBAL_ERROR = 1;
  return 0;
}

void write_int_tag(int address, uint32_t value)
{
  //unprotect?
  uint32_t result = 0;
  uint8_t tagWrite[4];
  uint32_t tempvalue = value;
  if((address % 4) == 0){
    for(uint8_t i =0; i<4; i++){
      tagWrite[i] = ((value) >> i*8) & 0xFF;
    }
    tag.writeEEPROM(address, tagWrite, 4);
    return;
  }
  GLOBAL_ERROR = 1;
}

void write_string_tag(int address, uint8_t * stringtowrite, uint8_t string_len)
{
  //unprotect?
  uint32_t result = 0;
  uint8_t tagWrite[4];
  uint32_t tempvalue = value;
  if((address % 4) == 0){
    tag.writeEEPROM(address, stringtowrite, string_len);
    return;
  }
  GLOBAL_ERROR = 1;
}

void read_string_tag(int address, uint8_t * stringtowrite, uint8_t string_len)
{
  //unprotect?
  uint32_t result = 0;
  uint8_t tagWrite[4];
  uint32_t tempvalue = value;
  if((address % 4) == 0){
    tag.readEEPROM(address, stringtowrite, string_len);
    return;
  }
  GLOBAL_ERROR = 1;
}


void setup_bmv080(){
  if (bmv080.begin(BMV080_ADDR, Wire) == false) {
      Serial.println("BMV080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
      while (1);
  }
  Serial.println("BMV080 found!");
  /* Initialize the Sensor (read driver, open, reset, id etc.)*/
  bmv080.init();

  /* Set the sensor Duty Cycling Period (seconds)*/
  uint16_t duty_cycling_period = 60;
  if(measurement_mode == 1){
    duty_cycling_period = 60;
  }
  else{
    duty_cycling_period = 300;
  }

  if(bmv080.setDutyCyclingPeriod(duty_cycling_period) == true)
  {
      Serial.println("BMV080 set to %d second duty cycle period", duty_cycling_period);
  }
  else
  {
      Serial.println("Error setting BMV080 duty cycle period");
      GLOBAL_ERROR = 1;
  }

  /* Set the sensor mode to Duty Cycle mode */
  if(bmv080.setMode(SF_BMV080_MODE_DUTY_CYCLE) == true)
  {
      Serial.println("BMV080 set to Duty Cycle mode");
  }
  else
  {
      Serial.println("Error setting BMV080 mode");
  }
}

void setup_tag(){
  if (!tag.begin(Wire))
  {
    Serial.println(F("ST25 not detected. Freezing..."));
    return;
  }

  Serial.println(F("ST25 connected."));

  // -=-=-=-=-=-=-=-=-

  Serial.println(F("Opening I2C security session with default password (all zeros)."));
  uint8_t password[8] = {0x0}; // Default password is all zeros
  tag.openI2CSession(password);

  Serial.print(F("I2C session is "));
  Serial.println(tag.isI2CSessionOpen() ? "opened." : "closed.");

  Serial.println(F("EEPROM area 1 will always be readable (datasheet page 60)."));
  Serial.print(F("EEPROM area 1 read protection: "));
  Serial.println(tag.getEEPROMReadProtectionBit(1) ? "protected." : "opened.");

  Serial.print(F("EEPROM area 1 write protection: "));
  Serial.println(tag.getEEPROMWriteProtectionBit(1) ? "protected." : "opened.");

  // -=-=-=-=-=-=-=-=-

  Serial.println(F("Protecting area 1 for write operation."));
  tag.programEEPROMWriteProtectionBit(1, true);

  bool area1WriteProtected = tag.getEEPROMWriteProtectionBit(1);
  Serial.print(F("EEPROM area 1 write protection: "));
  Serial.println(area1WriteProtected ? "protected." : "opened.");
  if (area1WriteProtected)
    Serial.println(F("Writing can only take place if I2C security session is opened."));

  // -=-=-=-=-=-=-=-=-

  // Read 16 bytes from EEPROM location 0x0
  uint8_t tagRead[16] = {0};
  Serial.print(F("Reading values, starting at location 0x0, with opened security session:        "));
  tag.readEEPROM(0x0, tagRead, 16); // Read the EEPROM: start at address 0x0, read contents into tagRead; read 16 bytes
  for (auto value : tagRead) // Print the contents
  {
    Serial.print(F("0x"));
    if (value < 0x10)
      Serial.print(F("0"));
    Serial.print(value, HEX);
    Serial.print(F(" "));
  }
  Serial.println();

  // -=-=-=-=-=-=-=-=-

  // Try to write 16 random bytes to EEPROM starting at location 0x0
  uint8_t tagWrite[16];
  randomSeed(analogRead(A0));
  for (uint8_t i = 0; i < 16; i++)
    tagWrite[i] = (uint8_t)random(0, 0xff);

  Serial.print(F("Writing random values, starting at location 0x0, with opened security session: "));
  for (auto value : tagWrite)
  {
    Serial.print(F("0x"));
    if (value < 0x10)
      Serial.print(F("0"));
    Serial.print(value, HEX);
    Serial.print(F(" "));
  }
  Serial.println();
  tag.writeEEPROM(0x0, tagWrite, 16);

  // -=-=-=-=-=-=-=-=-

  memset(tagRead, 0, 16);
  Serial.print(F("Reading values, starting at location 0x0, with opened security session:        "));
  tag.readEEPROM(0x0, tagRead, 16);
  for (auto value : tagRead)
  {
    Serial.print(F("0x"));
    if (value < 0x10)
      Serial.print(F("0"));
    Serial.print(value, HEX);
    Serial.print(F(" "));
  }
  Serial.println();

  // -=-=-=-=-=-=-=-=-

  Serial.println(F("Closing I2C security session - by writing the wrong password."));
  password[1] = {0x10}; // Change one byte of the password
  tag.openI2CSession(password);

  Serial.print(F("I2C session is "));
  Serial.println(tag.isI2CSessionOpen() ? "opened." : "closed.");

  // -=-=-=-=-=-=-=-=-

  // Try to write 16 random bytes from EEPROM location 0x0
  randomSeed(analogRead(A0));
  for (uint8_t i = 0; i < 16; i++)
    tagWrite[i] = (uint8_t)random(0, 0xff);

  Serial.print(F("Trying to write random values, starting at location 0x0, with closed security session: "));
  for (auto value : tagWrite)
  {
    Serial.print(F("0x"));
    if (value < 0x10)
      Serial.print(F("0"));
    Serial.print(value, HEX);
    Serial.print(F(" "));
  }
  Serial.println();
  tag.writeEEPROM(0x0, tagWrite, 16);

  // -=-=-=-=-=-=-=-=-

  Serial.print(F("Reading values, starting at location 0x0, with closed security session:                "));
  memset(tagRead, 0, 16);
  tag.readEEPROM(0x0, tagRead, 16);
  for (auto value : tagRead)
  {
    Serial.print(F("0x"));
    if (value < 0x10)
      Serial.print(F("0"));
    Serial.print(value, HEX);
    Serial.print(F(" "));
  }
  Serial.println();

  // -=-=-=-=-=-=-=-=-

  Serial.println(F("Re-opening I2C session - with the correct password."));
  password[1] = {0x0}; // Reset the password
  tag.openI2CSession(password);
  Serial.print(F("I2C session is "));
  Serial.println(tag.isI2CSessionOpen() ? "opened." : "closed.");

  // -=-=-=-=-=-=-=-=-

  Serial.println(F("Unprotecting area 1 for write operation."));
  tag.programEEPROMWriteProtectionBit(1, false);

  Serial.print(F("EEPROM area 1 write protection: "));
  Serial.println(tag.getEEPROMWriteProtectionBit(1) ? "protected." : "opened.");

  // -=-=-=-=-=-=-=-=-

  Serial.println(F("Closing I2C session - by writing the wrong password."));
  password[1] = {0x10}; // Change one byte of the password
  tag.openI2CSession(password);

  // -=-=-=-=-=-=-=-=-

  Serial.print(F("Writing zeros, starting at location 0x0, with closed session:  "));
  memset(tagWrite, 0, 16);
  for (auto value : tagWrite)
  {
    Serial.print(F("0x"));
    if (value < 0x10)
      Serial.print(F("0"));
    Serial.print(value, HEX);
    Serial.print(F(" "));
  }
  Serial.println();
  tag.writeEEPROM(0x0, tagWrite, 16);

  // -=-=-=-=-=-=-=-=-

  Serial.print(F("Reading values, starting at location 0x0, with closed session: "));
  memset(tagRead, 0, 16);
  tag.readEEPROM(0x0, tagRead, 16);
  for (auto value : tagRead)
  {
    Serial.print(F("0x"));
    if (value < 0x10)
      Serial.print(F("0"));
    Serial.print(value, HEX);
    Serial.print(F(" "));
  }
  Serial.println();

}


void setup()
{
    Serial.begin(115200);

    // while(!Serial) delay(10); // Wait for Serial to become available.
    // // Necessary for boards with native USB (like the SAMD51 Thing+).
    // // For a final version of a project that does not need serial debug (or a USB cable plugged in),
    // // Comment out this while loop, or it will prevent the remaining code from running.

    Wire.begin();
    setup_bmv080();
    setup_tag();

    init_memspace(); //on a fresh device:


}

void loop()
{
    if(bmv080.readSensor())
    {
        float pm25 = bmv080.PM25();  //µg/m³ teka spec says ,max is 7mg/m3 so 7000
        if(pm25 > 7000){
          pm25 = 7000;
        }
        uint16_t pm25_int = uint16_t (pm25);
        Serial.print(pm25_int);
        if(bmv080.isObstructed() == true)
        {
            Serial.print("\tObstructed");
            pm25_int = 0xffff;
        }
        //store this in the memory:
        current_data_add = find_write_addr(current_data_add);

        //find location to write
        write_int_tag(current_data_add + 1, 0xC1EAC1EA); //CLEAN means 
        write_int_tag(current_data_add, pm25_int);
        current_data_add++; //dont care about overflow, this is handled by the find_write_addr;

    }
    delay(1000);
}
