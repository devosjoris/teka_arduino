#include <Wire.h>
#include "SparkFun_BMV080_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BMV080

#include <SPI.h>
#include "epd2in66.h"
#include "imagedata.h"
#include "epdpaint.h"

#include "esp_sleep.h"
#include "esp_timer.h"

int64_t last_sensor_readout_us =0;


//RTC

uint32_t unix_timestamp = 1764430908;

#include <time.h>

// Formats: "YYYY-MM-DD HH:MM:SS"
void formatEpochSeconds(uint32_t epochSec, char* out, size_t outSize, bool useLocal = false) {
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
  snprintf(out, outSize, "%04d-%02d-%02d %02d:%02d:%02d",
           tmval.tm_year + 1900, tmval.tm_mon + 1, tmval.tm_mday,
           tmval.tm_hour, tmval.tm_min, tmval.tm_sec);
}

#define COLORED     0
#define UNCOLORED   1

UBYTE image[50*160];
Paint paint(image, 48, 160);    // width should be the multiple of 8 
UDOUBLE time_start_ms;
UDOUBLE time_now_s;



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

//Timestamp? phone connects -> writes timestamp to MEM_VAL_TIMESTAMP, and writes new timestamp flag
//esp32 boots -> 

//readout -> write as timestamp: absolute timestamp (when available from NFC)
//or a relative timestamp (seconds since last write?)



//ALL 32 bit numbers

#define MEM_PTR_TIMESTAMP           0
#define MEM_VAL_TIMESTAMP           4
#define MEM_PTR_LAST_WRITE          8
#define MEM_PTR_LAST_READ           12

#define MEM_VAL_MEASURE_MODE        16
#define MEM_VAL_WARNING             20    // MAX MIN(5000, MEM_VAL_LIMIT) 
#define MEM_VAL_LIMIT               24    // MAX 7000

#define MEM_VAL_NEWTIMESTAMP        28    //set by app
#define MEM_VAL_NEWSETTINGS         32    //set by app

#define MEM_VAL_USER_NAME_LENGTH    44
#define MEM_VAL_USER_NAME           48

#define MEM_VAL_DATA_VALID          92

#define MEM_VAL_DATA_START          (100 *2) 
#define MEM_VAL_DATA_END            (8188) //last bit

#define FW_REV                       1

int GLOBAL_ERROR =0;
uint16_t current_data_add     = MEM_VAL_DATA_START;
uint8_t measurement_mode      = 1;
uint8_t user_name_length      = 0;
uint8_t user_name[30];

uint32_t abs_x(int value){
  return (value < 0) ? -value : value;
}

uint16_t find_write_addr(uint16_t guess_addr){
  Serial.println("search start");
  Serial.println(guess_addr);
  for(uint16_t i =0; i< (60000/4); i=i+4){
    //Serial.println(i);
    uint32_t temp_addr = guess_addr +i;
    if(temp_addr >= MEM_VAL_DATA_END){
      temp_addr = (temp_addr - MEM_VAL_DATA_END) + MEM_VAL_DATA_START;
    }
    if(read_int_tag(temp_addr) == 0xC1EAC1EA){
      if(abs_x(temp_addr - read_int_tag(MEM_PTR_LAST_WRITE)) > 100){
        write_int_tag(MEM_PTR_LAST_WRITE, temp_addr);
      }
      return temp_addr;
    }
  }
  GLOBAL_ERROR = 1;
  return 0;

}

void reset_memspace(){
    Serial.print("INIT");
    write_int_tag(MEM_PTR_LAST_WRITE, MEM_VAL_DATA_START);
    write_int_tag(MEM_PTR_LAST_READ,  MEM_VAL_DATA_START);

    write_int_tag(MEM_VAL_DATA_START, 0xC1EAC1EA); //empty field -> ready to write:::
    
    write_int_tag(MEM_VAL_USER_NAME_LENGTH, 23);
    write_string_tag(MEM_VAL_USER_NAME, (uint8_t*) "USE APP TO SET USERNAME", 23);

    write_int_tag(MEM_VAL_MEASURE_MODE, 0x0001);     
    write_int_tag(MEM_VAL_WARNING, 5000);
    write_int_tag(MEM_VAL_LIMIT, 7000);

    write_int_tag(MEM_VAL_DATA_VALID, (((0x501d) << 16) + FW_REV));
}

void init_memspace(){
  if(read_int_tag(MEM_VAL_DATA_VALID) != (((0x501d) << 16) + FW_REV) ){ //not yet initialized
    reset_memspace();
  }
  Serial.print("RESTORE");
  current_data_add = read_int_tag(MEM_PTR_LAST_WRITE);
  current_data_add = find_write_addr(current_data_add);
  measurement_mode = read_int_tag(MEM_VAL_MEASURE_MODE);
  Serial.println("measurement_mode");
  Serial.println(measurement_mode);

  if(measurement_mode > 2) GLOBAL_ERROR = 1; //invalid value 
  user_name_length = read_int_tag(MEM_VAL_USER_NAME_LENGTH);
  if(user_name_length > 40) GLOBAL_ERROR = 1; //invalid value 
  else                      read_string_tag(MEM_VAL_USER_NAME_LENGTH, user_name, user_name_length);
  Serial.print("INIT/RESTORE DONE");
  if(GLOBAL_ERROR){
    reset_memspace();
  }
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
  if((address % 4) == 0){
    tag.writeEEPROM(address, stringtowrite, string_len);
    return;
  }
  GLOBAL_ERROR = 1;
}

void read_string_tag(int address, uint8_t * stringtoread, uint8_t string_len)
{
  //unprotect?
  if((address % 4) == 0){
    tag.readEEPROM(address, stringtoread, string_len);
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
  if(measurement_mode == 0){
    duty_cycling_period = 60;
  }
  else{
    duty_cycling_period = 300;
  }

  if(1){
    if(bmv080.setDutyCyclingPeriod(duty_cycling_period) == true)
    {
        Serial.println("BMV080 set to duty cycle period");
        Serial.println(duty_cycling_period);
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


}


void setup()
{
    Serial.begin(115200);
    Wire.begin();

    setup_tag();
    init_memspace(); //on a fresh device:

    setup_bmv080(); //do after tag since the memspace sets the duty cycle...

    Epd epd;
    Serial.print("e-Paper init...");
    if (epd.Init() != 0) {
      Serial.print("e-Paper init failed...");
      return;
    }
    Serial.print("2.66inch e-Paper demo...\r\n ");
    Serial.print("e-Paper Clear...\r\n ");
    epd.Clear();  

    paint.SetRotate(ROTATE_270);
    
  #if 1
    epd.Init_Partial();
    epd.Clear();
    Serial.print("partial display___ \r\n ");
    UBYTE i;
    time_start_ms = millis();
    for(i=0; i<10; i++) {
      char date_string[20];
      formatEpochSeconds(unix_timestamp, date_string, sizeof(date_string), false);

      paint.Clear(UNCOLORED);

      // date top-left (small font)
      paint.DrawStringAt(0, 0, date_string, &Font8, COLORED);

      // username roughly centered (16px/24px font, buffer 48x160)
      paint.DrawStringAt(20, 16, (char*)"joris", &Font16, COLORED);

      // smiley center-bottom
      paint.DrawStringAt(60, 32, ":)", &Font16, COLORED);

      Serial.print("refresh------\r\n ");
      epd.DisplayFrame_part(paint.GetImage(), 20, 50, 48, 160);  // UWORD Xstart, UWORD Ystart, UWORD iwidth, UWORD iheight
    }
  #endif


}

void loop()
{
    if(read_int_tag(MEM_VAL_NEWTIMESTAMP) == 0x0000501D){
      //new timestamp set by the app, add this timestamp to the ringbuffer...
      Serial.println("NEW_TIMESTAMP");
      current_data_add = find_write_addr(current_data_add);
      int timestamp = read_int_tag(MEM_VAL_TIMESTAMP); //this will alwasy have the lsb set, this is beeing handled by the app...
      if((timestamp %2 ==1) && timestamp > 1762359245){
        write_int_tag(current_data_add + 4, 0xC1EAC1EA);
        write_int_tag(current_data_add, timestamp);
        current_data_add=current_data_add + 4; //dont care about overflow, this is handled by the find_write_addr;
      }
      write_int_tag(MEM_VAL_NEWTIMESTAMP, 0xC1EAC1EA);

    }

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
            pm25_int = 0x0fff;
        }
        int64_t since_boot_us = esp_timer_get_time();
        //store this in the memory:
        current_data_add = find_write_addr(current_data_add);
        uint16_t relative_timestamp = ((since_boot_us -last_sensor_readout_us)/1000/1000/60);
        if(relative_timestamp < 1) relative_timestamp =1;
        if(relative_timestamp == 4) relative_timestamp =5;
        Serial.println("timestamp");
        Serial.println(relative_timestamp);
        //find location to write
        write_int_tag(current_data_add + 4, 0xC1EAC1EA); //CLEAN means 
        write_int_tag(current_data_add, (pm25_int << 1) +(relative_timestamp  << 16)); //lsb =1 means a timestamp value from nfc i.o. a sensor readout...
        last_sensor_readout_us = since_boot_us;
        current_data_add=current_data_add + 4; //dont care about overflow, this is handled by the find_write_addr;
    }
    Serial.print(".");
    delay(10000);
}
