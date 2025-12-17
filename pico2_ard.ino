#include "pinmap.h"

#include <Wire.h>
#include "SparkFun_BMV080_Arduino_Library.h" // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BMV080
#include <RV3028C7.h>


#include <SPI.h>
#include "epd2in66.h"
#include "imagedata.h"
#include "epdpaint.h"

#include "esp_sleep.h"
#include "esp_timer.h"

#include "tag_support.h"

#include <Preferences.h>

int64_t last_sensor_readout_us =0;
int64_t last_nvs_write_us = 0;

Preferences prefs;
static const char* NVS_NS = "senslog";
static const uint32_t NVS_MAGIC = 0x501D0001u;
static const uint16_t NVS_RING_SIZE = 256; // entries; each entry is one uint32_t packed value

#define SAD 0
#define HAPPY 1
#define NEUTRAL 2

//RTC
uint32_t unix_timestamp = 0;
RV3028C7 rtc;

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
  // snprintf(out, outSize, "%04d-%02d-%02d %02d:%02d:%02d",
  //          tmval.tm_year + 1900, tmval.tm_mon + 1, tmval.tm_mday,
  //          tmval.tm_hour, tmval.tm_min, tmval.tm_sec);

  snprintf(out, outSize, "%02d-%02d-%04d",
           tmval.tm_mday, tmval.tm_mon + 1, tmval.tm_year + 1900);
}

#define COLORED     0
#define UNCOLORED   1

#define SCREEN_WIDTH  296
#define SCREEN_HEIGHT 152

// Single full-screen paint buffer (logical space 152x296 with ROTATE_270)
// 152 * 296 pixels / 8 bits per byte = 5632 bytes
UBYTE image_full[152*296/8];

Paint paint(image_full, 152, 296);    // width must be multiple of 8
UDOUBLE time_start_ms;
UDOUBLE time_now_s;



SparkFunBMV080 bmv080; // Create an instance of the BMV080 class

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





int GLOBAL_ERROR =0;
uint16_t current_data_add     = MEM_VAL_DATA_START;
uint8_t measurement_mode      = 1;
uint8_t user_name_length      = 0;
uint8_t user_name[30];

void nvs_init()
{
  if (!prefs.begin(NVS_NS, false)) {
    Serial.println("NVS begin failed");
    GLOBAL_ERROR = 1;
    return;
  }

  uint32_t magic = prefs.getUInt("magic", 0);
  if (magic != NVS_MAGIC) {
    prefs.clear();
    prefs.putUInt("magic", NVS_MAGIC);
    prefs.putUShort("idx", 0);
    prefs.putUInt("last", 0);
  }
}

void nvs_log_packed(uint32_t packedValue)
{
  // Flash wear guard: write at most once per minute
  int64_t now_us = esp_timer_get_time();
  if (last_nvs_write_us != 0 && (now_us - last_nvs_write_us) < (60LL * 1000LL * 1000LL)) {
    return;
  }

  uint16_t idx = prefs.getUShort("idx", 0);
  char key[8];
  snprintf(key, sizeof(key), "d%03u", (unsigned)(idx % NVS_RING_SIZE));

  prefs.putUInt(key, packedValue);
  prefs.putUInt("last", packedValue);
  prefs.putUShort("idx", (uint16_t)((idx + 1) % NVS_RING_SIZE));

  last_nvs_write_us = now_us;
}

uint32_t abs_x(int value){
  return (value < 0) ? -value : value;
}

void drawStringCenter(Paint* p, sFONT* font, int box_x0, int box_y0, char* string_to_draw, int string_length, bool add_box) {
  int font_height = font->Height;
  int font_width  = font->Width; // average width per character from font

  int box_x1 = SCREEN_WIDTH - box_x0;
  int box_y1 = box_y0 + font_height + 2;

  if(add_box){
    p->DrawFilledRectangle(box_x0, box_y0, box_x1, box_y1, COLORED);
  }
  int text_x = (SCREEN_WIDTH - (font_width * string_length)) / 2;
  int text_y = box_y0 + 2;

  p->DrawStringAt(text_x, text_y, string_to_draw, font, add_box ? UNCOLORED : COLORED);
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
    if(read_int_tag(&tag, temp_addr) == 0xC1EAC1EA){
      if(abs_x(temp_addr - read_int_tag(&tag, MEM_PTR_LAST_WRITE)) > 100){
        write_int_tag(&tag, MEM_PTR_LAST_WRITE, temp_addr);
      }
      return temp_addr;
    }
  }
  GLOBAL_ERROR = 1;
  return 0;

}

void reset_memspace(){
    Serial.print("INIT");
  write_int_tag(&tag, MEM_PTR_LAST_WRITE, MEM_VAL_DATA_START);
  write_int_tag(&tag, MEM_PTR_LAST_READ,  MEM_VAL_DATA_START);

  write_int_tag(&tag, MEM_VAL_DATA_START, 0xC1EAC1EA); //empty field -> ready to write:::
    
  write_int_tag(&tag, MEM_VAL_USER_NAME_LENGTH, 23);
  write_string_tag(&tag, MEM_VAL_USER_NAME, (uint8_t*) "USE APP TO SET USERNAME", 23);

  write_int_tag(&tag, MEM_VAL_MEASURE_MODE, 0x0001);     
  write_int_tag(&tag, MEM_VAL_WARNING, 5000);
  write_int_tag(&tag, MEM_VAL_LIMIT, 7000);

  write_int_tag(&tag, MEM_VAL_DATA_VALID, (((0x501d) << 16) + FW_REV));
}

void init_memspace(){
  if(read_int_tag(&tag, MEM_VAL_DATA_VALID) != (((0x501d) << 16) + FW_REV) ){ //not yet initialized
    reset_memspace();
  }
  Serial.print("RESTORE");
  current_data_add = read_int_tag(&tag, MEM_PTR_LAST_WRITE);
  current_data_add = find_write_addr(current_data_add);
  measurement_mode = read_int_tag(&tag, MEM_VAL_MEASURE_MODE);
  Serial.println("measurement_mode");
  Serial.println(measurement_mode);

  if(measurement_mode > 2) GLOBAL_ERROR = 1; //invalid value 
  user_name_length = read_int_tag(&tag, MEM_VAL_USER_NAME_LENGTH);
  if(user_name_length > 40) GLOBAL_ERROR = 1; //invalid value 
  else                      read_string_tag(&tag, MEM_VAL_USER_NAME, user_name, user_name_length);
  Serial.print("INIT/RESTORE DONE");
  if(GLOBAL_ERROR){
    reset_memspace();
  }
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
        Serial.println("BMV080 set to duty cycle periodxxx");
        Serial.println(bmv080.dutyCyclingPeriod());
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

void drawSmiley(Paint* p, int cx, int cy, int radius, int smileytype) {
  // Face outline
  p->DrawCircle(cx, cy, radius, COLORED);

  // Optional: filled face
  // p->DrawFilledCircle(cx, cy, radius, COLORED);

  // Eyes (offset left/right and up)
  int eyeOffsetX = radius / 2;
  int eyeOffsetY = radius / 3;
  int eyeR = radius / 8;
  p->DrawFilledCircle(cx - eyeOffsetX, cy - eyeOffsetY, eyeR, COLORED);
  p->DrawFilledCircle(cx + eyeOffsetX, cy - eyeOffsetY, eyeR, COLORED);

  // Nose (small vertical line between eyes)
  int noseHeight = radius / 4;
  int noseX = cx;
  int noseYTop = cy - eyeOffsetY + eyeR + 1;
  int noseYBottom = noseYTop + noseHeight;
  for (int y = noseYTop; y <= noseYBottom; y++) {
    p->DrawPixel(noseX, y, COLORED);
  }

  // Mouth: approximate a smile with short lines forming an arc
  int mouthRadius = radius / 2;
  int mouthY;
  if(smileytype == HAPPY) mouthY = cy + radius / 4;
  else if(smileytype == SAD) mouthY = cy + radius / 2;
  else if(smileytype == NEUTRAL) mouthY = cy + radius / 3;

  for (int dx = -mouthRadius; dx <= mouthRadius; dx++) {
    int dy = (int)(0.4f * sqrt((float)(mouthRadius * mouthRadius - dx * dx)));
    if (smileytype == SAD) dy = -dy;
    if (smileytype == NEUTRAL) dy = 0;
    p->DrawPixel(cx + dx, mouthY + dy, COLORED);
    p->DrawPixel(cx + dx, mouthY + dy+1, COLORED); //2 line smile
  }
}

// Simple RGB LED control: values are 0 = off, non-zero = on
// NOTE: On this board only PIN_LED_B is a valid GPIO.
// PIN_LED_R and PIN_LED_G are currently mapped to invalid pins (47, 48)
// so we only drive the blue LED for now.
void setRgbLed(uint8_t r, uint8_t g, uint8_t b)
{
  (void)r;
  (void)g;
  digitalWrite(PIN_LED_B, (b == 0) ? LOW : HIGH);
}

// Initialize RV-3028 RTC, set it once to the compile time,
// and store the current Unix time in the global unix_timestamp.
void setup_rtc()
{
  while (rtc.begin() == false) {
    Serial.println("Failed to detect RV-3028-C7!");
    delay(5000);
  }
}


void synch_rtc(uint32_t secondsSinceEpoch){
  rtc.setUnixTimestamp(secondsSinceEpoch, true); // true to also set the esp32 internal timekeeping registers
  rtc.synchronize();
}


void i2c_scan() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    
    // Configure RGB LED pin (only blue is valid on this board)
    pinMode(PIN_CS_B, INPUT);  //unused but make sure it does not interfer with CS_A
    pinMode(PIN_LED_B, OUTPUT);
    setRgbLed(0, 0, 0); // start with LED off

    //power up the dcdc and the 3v3 regulator
    pinMode(PIN_DCDC_EN, OUTPUT);
    digitalWrite(PIN_DCDC_EN, HIGH);
    pinMode(PIN_V3V_CTRL, OUTPUT);
    digitalWrite(PIN_V3V_CTRL, HIGH);
    delay(1000); //let the voltages stabilize

    // Scan I2C bus and print all detected devices
    i2c_scan();

    // Initialize ESP32 internal persistent storage (NVS)
    nvs_init();


      // Initialize the RV-3028 RTC and set/read its time
    setup_rtc();
    // synch_rtc(1762359245); // Set to a fixed time for testing: 2024-09-05 12:00:45 UTC

    for(int i =0; i<5; i++){
      delay(1000);
      Serial.println(rtc.getCurrentDateTime());
      Serial.print(F("Unix timestamp: "));
      Serial.println(rtc.getUnixTimestamp());
    }


  setup_tag(&tag);
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
    paint.SetRotate(ROTATE_90);
    
  #if 1
    // epd.Init_Partial();
    // epd.Clear();
    Serial.print("full display___ \r\n ");
    UBYTE i;
    time_start_ms = millis();
    for(i=0; i<1; i++) {
      char date_string[20];
      formatEpochSeconds(unix_timestamp, date_string, sizeof(date_string), false);
      // Clear whole screen buffer
      paint.Clear(UNCOLORED);

      // Date: top-left
       paint.DrawStringAt(10, 3, date_string, &Font12, COLORED);

      int box_x0 = 9;
      int box_y0 = 30;

      drawStringCenter(&paint, &Font16, box_x0, box_y0, (char*)user_name, user_name_length, true);

      // // Smiley: center-bottom
      // // Place near bottom: y ~ SCREEN_HEIGHT * 2/3
      drawSmiley(&paint, SCREEN_WIDTH/5, SCREEN_HEIGHT * 2/3, 30, HAPPY);
      drawSmiley(&paint, SCREEN_WIDTH/2, SCREEN_HEIGHT * 2/3, 30, NEUTRAL);
      drawSmiley(&paint, 4*SCREEN_WIDTH/5, SCREEN_HEIGHT * 2/3, 30, SAD);

      Serial.print("refresh------\r\n ");
      // epd.DisplayFrame_part(paint.GetImage(),0,0,152,296);

      // for (int y = 0; y < paint.GetHeight(); y++) {
      //   for (int x = 0; x < paint.GetWidth(); x++) {
      //     int idx = (x + y * paint.GetWidth()) / 8;
      //     uint8_t mask = 0x80 >> (x % 8);

      //     bool bitIs1 = (paint.GetImage()[idx] & mask) != 0;

      //     // With IF_INVERT_COLOR=1 in epdpaint.cpp:
      //     // colored=1 sets bit to 1, colored=0 clears to 0.
      //     // In your sketch: COLORED=0 (black), UNCOLORED=1 (white)
      //     bool isWhite = bitIs1;
      //     Serial.print(isWhite ? ' ' : '.'); // '.' = black pixel
      //   }
      //   Serial.println();
      // }
      epd.DisplayFrame(paint.GetImage());

      // write_int_tag(&tag, MEM_VAL_TIMESTAMP, 1765989831);
      // write_int_tag(&tag, MEM_VAL_NEWTIMESTAMP, 0x0000501D);
    }
    // epd.Sleep();
  #endif


}

void loop()
{
    //phone sets new timestamp on connect -> ...
    if(read_int_tag(&tag, MEM_VAL_NEWTIMESTAMP) == 0x0000501D){
      //new timestamp set by the app, add this timestamp to the ringbuffer...
      Serial.println("NEW_TIMESTAMP");
      current_data_add = find_write_addr(current_data_add);
      int timestamp = read_int_tag(&tag, MEM_VAL_TIMESTAMP); //this will alwasy have the lsb set, this is beeing handled by the app...
      if((timestamp %2 ==1) && timestamp > 1762359245){
        // Use the received Unix timestamp to set the RTC absolute time.
        // The LSB is used as a marker, so mask it off for the RTC.
        uint32_t epochSec = ((uint32_t)timestamp);
        synch_rtc(epochSec);
        unix_timestamp = epochSec;

        write_int_tag(&tag, current_data_add + 4, 0xC1EAC1EA);
        write_int_tag(&tag, current_data_add, timestamp);
        current_data_add=current_data_add + 4; //dont care about overflow, this is handled by the find_write_addr;
      }
      write_int_tag(&tag, MEM_VAL_NEWTIMESTAMP, 0xC1EAC1EA);
    }

    if(bmv080.readSensor())
    {
        float pm25 = bmv080.PM25();  //µg/m³ teka spec says ,max is 7mg/m3 so 7000
        Serial.println("FLOAT PM2.5: ");
        Serial.print(pm25);
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
        write_int_tag(&tag, current_data_add + 4, 0xC1EAC1EA); //CLEAN means 
        uint32_t packed = (uint32_t)(pm25_int << 1) + (uint32_t)(relative_timestamp << 16);
        write_int_tag(&tag, current_data_add, packed); //lsb =1 means a timestamp value from nfc i.o. a sensor readout...
        // Also store into ESP32 internal flash (NVS) for redundancy
        nvs_log_packed(packed);
        last_sensor_readout_us = since_boot_us;
        current_data_add=current_data_add + 4; //dont care about overflow, this is handled by the find_write_addr;
    }
    Serial.print(".");
    delay(10000);
}
