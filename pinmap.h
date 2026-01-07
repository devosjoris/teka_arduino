#ifndef PINMAP_H
    #define PINMAP_H


    #define PIN_IRQ             5
    #define PIN_RTC_CHARGE      6
    // #define PIN_VBAT_RD         7
    #define PIN_CS_A            15
    #define PIN_DC              16
    #define PIN_RST             17
    #define PIN_BUSY            18
    #define PIN_SDA             8
    #define PIN_SCL             9
    #define PIN_CS_B            10
    #define PIN_DIN             11
    #define PIN_SCLK            12

    #define PIN_LED_B           14
    #define PIN_WP              21
    #define PIN_LED_G           47
    #define PIN_LED_R           48

    #define PIN_IO0             0
    #define PIN_BUT1            38



    #define PIN_DCDC_EN         2
    #define PIN_V3V_CTRL        1




    //ALL 32 bit numbers

    #define MEM_PTR_TIMESTAMP           0
    #define MEM_VAL_TIMESTAMP           4
    #define MEM_PTR_LAST_WRITE          8
    #define MEM_PTR_LAST_READ           12

    #define MEM_VAL_MEASURE_MODE        16
    #define MEM_VAL_WARNING             20    // MAX MIN(5000, MEM_VAL_LIMIT) 
    #define MEM_VAL_LIMIT               24    // MAX 7000

    #define MEM_VAL_NEWSETTINGS         32    //set by app
    #define MEM_VAL_NEWTIMESTAMP        36    //set by app

    #define MEM_VAL_USER_NAME_LENGTH    44
    #define MEM_VAL_USER_NAME           48

    #define MEM_VAL_DATA_VALID          92

    #define MEM_VAL_DATA_START          (100 *2) 
    #define MEM_VAL_DATA_END            (8188) //last bit

    #define FW_REV                       1



// I2C device found at address 0x2D  !  nfc
// I2C device found at address 0x52  !  rtc
// I2C device found at address 0x53  !  nfc
// I2C device found at address 0x57  ! nfc

    #define RTC_I2C_ADDR 0x52   //only one we find when disabling the ldo/dcdc since connected straight to vbat
    #define BMV080_ADDR 0x56   // SparkFun BMV080 Breakout defaults to 0x57 (TBD)

    #define EEPROM_I2C_ADDR 0x53 
    // For most I²C library implementations, the relevant 7-bit addresses are:
    // User Memory Area: 0x53 (hex)
    // System Configuration Area / Registers: 0x57 (hex) 
    //todo conflice with BMV080?

    #define ST25_ADDR 0x57  // SparkFun BMV080 Breakout defaults to 0x57
                            //so conflicts with the nfc!!!



#endif