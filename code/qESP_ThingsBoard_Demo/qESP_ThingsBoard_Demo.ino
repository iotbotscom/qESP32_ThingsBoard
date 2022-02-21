/*****************************************************************************
  This is the Demo of using qESP / qESP32 / qButton IOT boards

  Hardware Setup (qESP / qESP32):
    qESP / qESP32 / qButton IOT board;
    ESP programmer:
        option1 : qProg with USB cable;
        option2 : any ESP chip programmer with jump wires and USB cable;
    CR123A Lithium Battery.

  Hardware Setup (qButton):
    qButton IOT board;
    USB Cable.

  Arduino Setup:
    "ESP32 Dev Module" to be choosen as board;
    Arduino Libs to be installed:
        Adafruit_BME280_Library : https://github.com/adafruit/Adafruit_BME280_Library
        Adafruit-PWM-Servo-Driver-Library : https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
        Adafruit_TSL2591_Library : https://github.com/adafruit/Adafruit_TSL2591_Library
        ThingsBoard related: https://thingsboard.io/docs/samples/esp32/gpio-control-pico-kit-dht22-sensor/

  Project Options:
    You need to set these defines in user_config.h file:
      - QESP_IOT_BOARD_TYPE : CPU Board type, "qESP32" Default
      - QESP_IOT_BOARD_SLEEP_MODE_TYPE : Project Sleep Mode Option, "Always On" Default

  ThingsBoard Demo Source code:
    https://github.com/iotbotscom/qESP32_ThingsBoard

====================================================================================================
Revision History:

Author                          Date                Revision NUmber          Description of Changes
------------------------------------------------------------------------------------

iotbotscom                02/18/2022               1.0.0                        Initial release
iotbotscom                02/21/2022               1.0.1                        Battery Reading was added

*****************************************************************************/

/*---- Include files -------------------------------------------------------------------*/
#include "user_config.h"
#include "hw_config.h"
#include "credentials.h"
#ifdef ESP32
  #include <WiFi.h>
  #include "driver/adc.h"
#else
  #include <ESP8266WiFi.h>
#endif
#include <ArduinoJson.h>
#include <Adafruit_BME280.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_TSL2591.h>
#include <ThingsBoard.h>

/*---- Defines  ------------------------------------------------------------------------*/
//I2C Addresses
#define BME280_ADDRESS_I2C      0x76
#define LIS3DH_ADDRESS_I2C      0x19
#define TSL2591_ADDRESS_I2C     0x29
#define PCA9685_ADDRESS_I2C     0x40

// Motion Sensor REGs
#define LIS3DH_STATUS_REG_AUX_ADDR   0x07
#define LIS3DH_OUT_ADC1_L_ADDR       0x08
#define LIS3DH_OUT_ADC1_H_ADDR       0x09
#define LIS3DH_OUT_ADC2_L_ADDR       0x0a
#define LIS3DH_OUT_ADC2_H_ADDR       0x0b
#define LIS3DH_OUT_ADC3_L_ADDR       0x0c
#define LIS3DH_OUT_ADC3_H_ADDR       0x0d
#define LIS3DH_INT_COUNTER_REG_ADDR  0x0e
#define LIS3DH_WHO_AM_I_ADDR         0x0F
#define LIS3DH_TEMP_CFG_ADDR         0x1F
#define LIS3DH_CTRL_REG1_ADDR        0x20
#define LIS3DH_CTRL_REG2_ADDR        0x21
#define LIS3DH_CTRL_REG3_ADDR        0x22
#define LIS3DH_CTRL_REG4_ADDR        0x23
#define LIS3DH_CTRL_REG5_ADDR        0x24
#define LIS3DH_CTRL_REG6_ADDR        0x25
#define LIS3DH_REFERENCE_ADDR        0x26
#define LIS3DH_STATUS_REG_ADDR       0x27
#define LIS3DH_OUT_X_L_ADDR          0x28
#define LIS3DH_OUT_X_H_ADDR          0x29
#define LIS3DH_OUT_Y_L_ADDR          0x2A
#define LIS3DH_OUT_Y_H_ADDR          0x2B
#define LIS3DH_OUT_Z_L_ADDR          0x2C
#define LIS3DH_OUT_Z_H_ADDR          0x2D
#define LIS3DH_FIFO_CTRL_REG_ADDR    0x2E
#define LIS3DH_FIFO_SRC_REG_ADDR     0x2F
#define LIS3DH_INT1_CFG_ADDR         0x30
#define LIS3DH_INT1_SRC_ADDR         0x31
#define LIS3DH_INT1_THS_ADDR         0x32
#define LIS3DH_INT1_DURATION_ADDR    0x33
#define LIS3DH_INT2_CFG_ADDR         0x34
#define LIS3DH_INT2_SRC_ADDR         0x35
#define LIS3DH_INT2_THS_ADDR         0x36
#define LIS3DH_INT2_DURATION_ADDR    0x37
#define LIS3DH_CLICK_CFG_ADDR        0x38
#define LIS3DH_CLICK_SRC_ADDR        0x39
#define LIS3DH_CLICK_THS_ADDR        0x3A
#define LIS3DH_TIME_LIMIT_ADDR       0x3B
#define LIS3DH_TIME_LATENCY_ADDR     0x3C
#define LIS3DH_TIME_WINDOW_ADDR      0x3D
#define LIS3DH_ACT_THS_ADDR          0x3E
#define LIS3DH_INACT_DUR_ADDR        0x3F

#define MOTION_THRESHOLD            6
#define MOTION_DURATION             10

// RGB Leds
#define RGB_LED1_CHANNEL_RED    2
#define RGB_LED1_CHANNEL_GREEN  3
#define RGB_LED1_CHANNEL_BLUE   4

#define RGB_LED2_CHANNEL_RED    5
#define RGB_LED2_CHANNEL_GREEN  6
#define RGB_LED2_CHANNEL_BLUE   7

#define RGB_LED3_CHANNEL_RED    14
#define RGB_LED3_CHANNEL_GREEN  1
#define RGB_LED3_CHANNEL_BLUE   0

#define RGB_LED4_CHANNEL_RED    11
#define RGB_LED4_CHANNEL_GREEN  12
#define RGB_LED4_CHANNEL_BLUE   13

#define RGB_LED5_CHANNEL_RED    8
#define RGB_LED5_CHANNEL_GREEN  9
#define RGB_LED5_CHANNEL_BLUE   10

//ADC
#define ADC_BUF_MAX             8

#if ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QESP32))
#define ADC_BATTERY_VOLTAGE_NORMAL          3000 // 3000mV
#define ADC_POWER_DOWN_VOLTAGE_TRESHOULD    2600 // 2600mV

#define ADC_REFERENCE_VOLTAGE               3300 // 1100mV
#define ADC_CODE_MAX                        4095 // 12 bits
#define ADC_FULL_SCALE                      4096.0 // 12 bits
#define ADC_DIVIDER_VALUE                   2.3 // 10k / 10k (Trimmed)
#elif ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QBUTTON))
#define ADC_BATTERY_VOLTAGE_NORMAL          4200 // 3000mV
#define ADC_POWER_DOWN_VOLTAGE_TRESHOULD    3400 // 2600mV

#define ADC_REFERENCE_VOLTAGE               3300 // 1100mV
#define ADC_CODE_MAX                        4095 // 12 bits
#define ADC_FULL_SCALE                      4096.0 // 12 bits
#define ADC_DIVIDER_VALUE                   1.45 // 10k / 20k (Trimmed)
#else
#define ADC_BATTERY_VOLTAGE_NORMAL          3000 // 3000mV
#define ADC_POWER_DOWN_VOLTAGE_TRESHOULD    2600 // 2600mV

#define ADC_REFERENCE_VOLTAGE               1000 // 1000mV
#define ADC_CODE_MAX                        1023 // 10 bits
#define ADC_FULL_SCALE                      1024.0 // 10 bits
#define ADC_DIVIDER_VALUE                   4.15 // 330k / 100k (Trimmed)
#endif


/*---- Typedefs ------------------------------------------------------------------------*/

/*---- Variables -----------------------------------------------------------------------*/
// Devices status
int sleep_mode = QESP_IOT_BOARD_SLEEP_MODE_TYPE;
int sleep_timer_sec = 600; // sec
bool is_bme280_on = false;
bool is_lis3dh_on = false;
bool is_tsl2591_on = false;
bool is_pca9685_on = false;

// Device Sensor Data
float bme280_temperature = 0;
float bme280_humidity = 0;
float bme280_pressure = 0;
uint16_t tsl2591_ir = 0;
uint16_t tsl2591_full = 0;
uint16_t tsl2591_visible = 0;
uint16_t tsl2591_lux = 0;
int16_t lis3dh_data_x =0;
int16_t lis3dh_data_y =0;
int16_t lis3dh_data_z =0;
int battery_mV = 0;
int battery_percent = 0;

// Interrupts
bool user_key_interrupt_status = false;
bool xint_interrupt_status = false;
bool xint2_interrupt_status = false;

// Sensors
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_ADDRESS_I2C, Wire);
Adafruit_BME280 bme280;
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591);

// ADC
uint16_t adc_buf[ADC_BUF_MAX];
uint8_t adc_buf_idx;

// WiFi
int8_t rssi = 0;

// WiFi Client
WiFiClient qESPClient;

// Cloud
ThingsBoard tb(qESPClient);


/*---- Function prototypes -------------------------------------------------------------*/
void hal_init(void );
void i_am_alive(void );
void take_a_breath(void );
void collect_my_senses(void );
void get_my_senses(void );
void share_my_senses(void );
void go_to_bed(void );
void sysled_blink(void );

void bme280_get_data(void );
void tsl2591_get_data(void );
void lis3dh_get_data(void );
void get_battery(void );

bool lis3dh_read_reg(uint8_t RegisterAddr, uint8_t NumByteToRead, uint8_t * p_RegisterValue);
bool lis3dh_write_reg(uint8_t RegisterAddr, uint8_t NumByteToWrite, uint8_t * p_RegisterValue);

void ICACHE_RAM_ATTR user_key_interrupt(void );
void ICACHE_RAM_ATTR xint_interrupt(void );
void ICACHE_RAM_ATTR xint2_interrupt(void );

/*---- Function declarations------------------------------------------------------------*/
void setup()
{
    // Set console baud rate
    Serial.begin(115200);

    Serial.print("\r\n***************************************************************************************************\r\n");
    Serial.print("\r\n **** IOT-BOTS.COM **** \r\n");
#if ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QESP))
    Serial.print("\r\n **** ThingsBoard Demo : qESP IOT Board **** \r\n");
#elif ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QESP32))
    Serial.print("\r\n **** ThingsBoard Demo : qESP32 IOT Board **** \r\n");
#elif ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QBUTTON))
    Serial.print("\r\n **** ThingsBoard Demo : qButton IOT Board **** \r\n");
#else
    Serial.print("\r\n **** ThingsBoard Demo : NO IOT Board Defined **** \r\n");
    Serial.print("\r\n***************************************************************************************************\r\n");

    while(1)
    {
        delay(1000);
    }
#endif
    Serial.print("\r\n***************************************************************************************************\r\n");

    Serial.print("\r\n---- Setup Started ----\r\n");

    // HAL Init
    hal_init();

    // RGB Check
    take_a_breath();

    // Sound Check
    i_am_alive();

    // Sensor Init
    collect_my_senses();

    Serial.println("\r\n---- Setup Done ----\r\n");
}

///////////////////////////////////////////////////////////////////////////////
void loop()
{
    Serial.print("\r\n---- Loop Started ----\r\n");

    // Collect Sensor Data
    get_my_senses();

    // Publish Data
    share_my_senses();

    // Put Controller to Sleep Mode
    go_to_bed();

    Serial.println("\r\n---- Loop Done ----\r\n");
}

///////////////////////////////////////////////////////////////////////////////
void hal_init(void )
{
    // TPS61291DRV DC/DC On : The first thing to do!
    pinMode(HIGH_PWR_ON_PIN, OUTPUT);
    digitalWrite(HIGH_PWR_ON_PIN, HIGH);

    // User Button & LED : share the same pin
    pinMode(K1_SYSLED_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(K1_SYSLED_PIN), user_key_interrupt, FALLING);
    user_key_interrupt_status = false;

    // XINT Motion Sensor Interrupt
    pinMode(XINT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(XINT_PIN), xint_interrupt, RISING);
    xint_interrupt_status = false;

    // XINT2 Motion Sensor Interrupt
    pinMode(XINT2_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(XINT2_PIN), xint2_interrupt, RISING);
    xint2_interrupt_status = false;

    // Battery Measurement Enable
    pinMode(LOW_BAT_EN_PIN, OUTPUT);
    digitalWrite(LOW_BAT_EN_PIN, HIGH);

    // Beeper
    pinMode(AUDIO_PIN, OUTPUT);
    digitalWrite(AUDIO_PIN, LOW);

    // Load (RGB LEDs & Speaker) Enable
    pinMode(LOAD_EN_PIN, OUTPUT);
    digitalWrite(LOAD_EN_PIN, HIGH);

#if ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QESP))
    Wire.pins(I2C_SDA, I2C_SCL);
    Wire.begin();                       // Wire must be started first
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz
#else
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);              // Supported baud rates are 100kHz, 400kHz, and 1000kHz
#endif
}

void i_am_alive(void )
{
    int i;

    // Turn Load On
    digitalWrite(LOAD_EN_PIN, LOW);

    for(i = 0; i < 500; i++)
    {
        digitalWrite(AUDIO_PIN, HIGH);
        delayMicroseconds(125);
        digitalWrite(AUDIO_PIN, LOW);
        delayMicroseconds(125);
    }

    // Turn Load Off
    digitalWrite(LOAD_EN_PIN, HIGH);
}

void take_a_breath(void )
{
    int i;

    pca9685.begin();
    pca9685.setOscillatorFrequency(27000000);
    pca9685.setPWMFreq(1600);

    for(i = 0; i < 16; i++)
    {
        pca9685.setPWM(i, 0, 4095);
    }

    // Turn Load On
    digitalWrite(LOAD_EN_PIN, LOW);

    // Red LEDs On
    for(i = 0; i < 4096; i += 8)
    {
        pca9685.setPWM(RGB_LED1_CHANNEL_RED, 0, 4095 - i);
        pca9685.setPWM(RGB_LED2_CHANNEL_RED, 0, 4095 - i);
        pca9685.setPWM(RGB_LED3_CHANNEL_RED, 0, 4095 - i);
        pca9685.setPWM(RGB_LED4_CHANNEL_RED, 0, 4095 - i);
        pca9685.setPWM(RGB_LED5_CHANNEL_RED, 0, 4095 - i);

#ifdef ESP8266
        yield();  // take a breather, required for ESP8266
#endif
    }

    delay(500);

    // Red LEDs Off
    for(i = 0; i < 4096; i += 8)
    {
        pca9685.setPWM(RGB_LED1_CHANNEL_RED, 0, i);
        pca9685.setPWM(RGB_LED2_CHANNEL_RED, 0, i);
        pca9685.setPWM(RGB_LED3_CHANNEL_RED, 0, i);
        pca9685.setPWM(RGB_LED4_CHANNEL_RED, 0, i);
        pca9685.setPWM(RGB_LED5_CHANNEL_RED, 0, i);

#ifdef ESP8266
        yield();  // take a breather, required for ESP8266
#endif
    }

    delay(500);

    // Green LEDs On
    for(i = 0; i < 4096; i += 8)
    {
        pca9685.setPWM(RGB_LED1_CHANNEL_GREEN, 0, 4095 - i);
        pca9685.setPWM(RGB_LED2_CHANNEL_GREEN, 0, 4095 - i);
        pca9685.setPWM(RGB_LED3_CHANNEL_GREEN, 0, 4095 - i);
        pca9685.setPWM(RGB_LED4_CHANNEL_GREEN, 0, 4095 - i);
        pca9685.setPWM(RGB_LED5_CHANNEL_GREEN, 0, 4095 - i);

#ifdef ESP8266
        yield();  // take a breather, required for ESP8266
#endif
    }

    delay(500);

    // Green LEDs Off
    for(i = 0; i < 4096; i += 8)
    {
        pca9685.setPWM(RGB_LED1_CHANNEL_GREEN, 0, i);
        pca9685.setPWM(RGB_LED2_CHANNEL_GREEN, 0, i);
        pca9685.setPWM(RGB_LED3_CHANNEL_GREEN, 0, i);
        pca9685.setPWM(RGB_LED4_CHANNEL_GREEN, 0, i);
        pca9685.setPWM(RGB_LED5_CHANNEL_GREEN, 0, i);

#ifdef ESP8266
        yield();  // take a breather, required for ESP8266
#endif
    }

    delay(500);

    // Blue LEDs On
    for(i = 0; i < 4096; i += 8)
    {
        pca9685.setPWM(RGB_LED1_CHANNEL_BLUE, 0, 4095 - i);
        pca9685.setPWM(RGB_LED2_CHANNEL_BLUE, 0, 4095 - i);
        pca9685.setPWM(RGB_LED3_CHANNEL_BLUE, 0, 4095 - i);
        pca9685.setPWM(RGB_LED4_CHANNEL_BLUE, 0, 4095 - i);
        pca9685.setPWM(RGB_LED5_CHANNEL_BLUE, 0, 4095 - i);

#ifdef ESP8266
        yield();  // take a breather, required for ESP8266
#endif
    }

    delay(500);

    // Blue LEDs Off
    for(i = 0; i < 4096; i += 8)
    {
        pca9685.setPWM(RGB_LED1_CHANNEL_BLUE, 0, i);
        pca9685.setPWM(RGB_LED2_CHANNEL_BLUE, 0, i);
        pca9685.setPWM(RGB_LED3_CHANNEL_BLUE, 0, i);
        pca9685.setPWM(RGB_LED4_CHANNEL_BLUE, 0, i);
        pca9685.setPWM(RGB_LED5_CHANNEL_BLUE, 0, i);

#ifdef ESP8266
        yield();  // take a breather, required for ESP8266
#endif
    }

    // Turn Load Off
    digitalWrite(LOAD_EN_PIN, HIGH);
}

void collect_my_senses(void )
{
    uint8_t val;

    // BME280
    if(bme280.begin(BME280_ADDRESS_I2C))
    {
/*
    bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::FILTER_OFF);
*/
        Serial.println("BME280 Sensor Found!");
        is_bme280_on = true;
    }
    else
    {
        Serial.println("No BME280 Sensor Found!");
        is_bme280_on = false;
    }

    // TSL2591
    if(tsl2591.begin(&Wire, TSL2591_ADDRESS_I2C))
    {
        Serial.println("TSL2591 Sensor Found!");

        tsl2591.setGain(TSL2591_GAIN_MED);

        tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);

        is_tsl2591_on = true;
    }
    else
    {
        Serial.println("No TSL2591 Sensor Found!");
        is_tsl2591_on = false;
    }

    // LIS3DH
    lis3dh_read_reg(LIS3DH_WHO_AM_I_ADDR, &val);

    if(val == 0x33)
    {
        Serial.println("LIS3DH Sensor Found!");

        // 50Hz / XYZ Enabled
        lis3dh_write_reg(LIS3DH_CTRL_REG1_ADDR, 0x27);

        // High-pass filter Enabled for both INTs
        lis3dh_write_reg(LIS3DH_CTRL_REG2_ADDR, 0x05);

        // I1 CLICK & IA1 Enabled
        lis3dh_write_reg(LIS3DH_CTRL_REG3_ADDR, 0xc0);

        // �2 g  & High-resolution mode On
        lis3dh_write_reg(LIS3DH_CTRL_REG4_ADDR, 0x08);

        // INT_POLARITY active level
        lis3dh_write_reg(LIS3DH_CTRL_REG6_ADDR, 0x00);

        // Reference value  = 0
        lis3dh_write_reg(LIS3DH_REFERENCE_ADDR, 0x00);

        // Temp & ADC disabled
        lis3dh_write_reg(LIS3DH_TEMP_CFG_ADDR, 0x00);

        lis3dh_write_reg(LIS3DH_CLICK_THS_ADDR, MOTION_THRESHOLD);
        lis3dh_write_reg(LIS3DH_TIME_LIMIT_ADDR, MOTION_DURATION);

        lis3dh_write_reg(LIS3DH_CLICK_CFG_ADDR, 0x15);
        lis3dh_write_reg(LIS3DH_CTRL_REG5_ADDR, 0x08);

        lis3dh_read_reg(LIS3DH_INT1_SRC_ADDR, &val);
        lis3dh_read_reg(LIS3DH_CLICK_SRC_ADDR, &val);
        lis3dh_read_reg(LIS3DH_INT2_SRC_ADDR, &val);

        is_lis3dh_on = true;
    }
    else
    {
        Serial.println("No LIS3DH Sensor Found!");
        is_lis3dh_on = false;
    }
}

void get_my_senses(void )
{
    if(is_bme280_on == true)
    {
        bme280_get_data();
    }

    if(is_tsl2591_on == true)
    {
        tsl2591_get_data();
    }

    if(is_lis3dh_on == true)
    {
        lis3dh_get_data();
    }

    get_battery();

    if(user_key_interrupt_status == true)
    {
        user_key_interrupt_status = false;

        Serial.println("\r\n-> USER_KEY Interrupt");
    }

    if(xint_interrupt_status == true)
    {
        xint_interrupt_status = false;

        sysled_blink();

        Serial.println("\r\n-> Motion XINT Interrupt");
    }

    if(xint2_interrupt_status == true)
    {
        xint2_interrupt_status = false;

        sysled_blink();

        Serial.println("\r\n-> Motion XINT2 Interrupt");
    }
}

void share_my_senses(void )
{
    uint8_t attemps = 0;

    // WiFi
    if(WiFi.status() != WL_CONNECTED)
    {
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        delay(100);

        // Connecting to WiFi
        Serial.print("\r\nConnecting to:\r\n");

        Serial.print("WiFi SSID: ");
        Serial.println((const char *)WIFI_SSID);
        Serial.print("WiFi Pass: ");
        Serial.println((const char *)WIFI_PASS);

        WiFi.begin((const char *)WIFI_SSID, (const char *)WIFI_PASS);

        while(WiFi.status() != WL_CONNECTED)
        {
            Serial.print(".");
            sysled_blink();
            delay(1000);

            if(attemps++ > 30)
            {
                Serial.print("\r\nWiFi Failed to connect\r\n");

                return;
            }
        }

        Serial.print("\r\nWiFi connected\r\n");

        Serial.println("\r\nWiFi: ");
        Serial.print(" IP address: ");
        Serial.println(WiFi.localIP());

        rssi = (int8_t)WiFi.RSSI();
        Serial.print(" RSSI: ");
        Serial.print(rssi);
        Serial.println("dBm");
    }
    else
    {
        rssi = (int8_t)WiFi.RSSI();
        Serial.print(" RSSI: ");
        Serial.print(rssi);
        Serial.println("dBm");
    }

    // ThingsBoard Cloud part
    if(!tb.connected())
    {
        // Connecting to the ThingsBoard
        Serial.print("\r\nConnecting to:\r\n");

        Serial.print("Cloud: ");
        Serial.println((const char *)CLOUD_THINGSBOARD_SERVER);
        Serial.print("Token: ");
        Serial.println((const char *)CLOUD_THINGSBOARD_TOKEN);

        if(!tb.connect(CLOUD_THINGSBOARD_SERVER, CLOUD_THINGSBOARD_TOKEN))
        {
            Serial.print("\r\nCloud Failed to connect\r\n");

            return;
        }
        else
        {
            Serial.print("\r\nCloud Connected\r\n");
        }
    }

    // Publish Data
    tb.sendTelemetryFloat("Temperature", bme280_temperature);
    tb.sendTelemetryFloat("Humidity", bme280_humidity);
    tb.sendTelemetryFloat("Pressure", bme280_temperature);
    tb.sendTelemetryInt("Light-IR", tsl2591_ir);
    tb.sendTelemetryInt("Light-VIS", tsl2591_visible);
    tb.sendTelemetryInt("Light-LUX", tsl2591_lux);
    tb.sendTelemetryInt("Motion-X", lis3dh_data_x);
    tb.sendTelemetryInt("Motion-Y", lis3dh_data_y);
    tb.sendTelemetryInt("Motion-Z", lis3dh_data_z);
    tb.sendTelemetryInt("Battery_mV", battery_mV);
    tb.sendTelemetryInt("Battery_percent", battery_percent);

    Serial.print("\r\nData Published\r\n");

}

void go_to_bed(void )
{
    uint64_t bit_mask = 0; // No interrupts

    if(sleep_mode == QESP_IOT_BOARD_SLEEP_MODE_ALWAYS_ON)
    {
        // Just to have 1 sec timeout to run loop again
        delay(1000);

        return;
    }
    else
    {
        delay(1000);

        // Sensors Sleep (TBD)
        pca9685.sleep();

        // ESP Sleep

        Serial.println("\r\nSleep Mode :");

        // Timer
        if(sleep_mode & QESP_IOT_BOARD_SLEEP_MODE_TIMER)
        {
            Serial.println(" - Timer");

            esp_sleep_enable_timer_wakeup(sleep_timer_sec * 1000 * 1000);
        }

        // User Button, K1 Pin
        if(sleep_mode & QESP_IOT_BOARD_SLEEP_MODE_USER_KEY)
        {
            Serial.println(" - User Button");

            // EXT0
            esp_sleep_enable_ext0_wakeup((gpio_num_t)K1_PIN, 0);
        }

        // Motion, XINT Pin
        if(sleep_mode & QESP_IOT_BOARD_SLEEP_MODE_MOTION)
        {
            Serial.println(" - Motion");

            // EXT1
            bit_mask = 0x0100000000; // XINT_PIN (32)

            esp_sleep_enable_ext1_wakeup(bit_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
        }

        adc_power_off(); // Some ESP32 hack (#1113)

        Serial.println("\r\nMove to Sleep");

        // TPS61291DRV DC/DC Off
        digitalWrite(HIGH_PWR_ON_PIN, LOW);

        esp_deep_sleep_start();
    }
}

void bme280_get_data(void )
{
    Serial.println("\r\nBME280 Sensor: ");

    //get and print temperatures
    Serial.print(" Temperature, F: ");
    bme280_temperature = bme280.readTemperature();
    bme280_temperature = ((bme280_temperature * 9 / 5) + 32);
    Serial.println(bme280_temperature);

    //get and print atmospheric bme280_pressure data
    Serial.print(" Pressure, hPa: ");
    Serial.println(bme280_pressure = bme280.readPressure() / 100.0);

    //get and print bme280_humidity data
    Serial.print(" Humidity, %: ");
    Serial.println(bme280_humidity = bme280.readHumidity());
}

void tsl2591_get_data(void )
{
    uint32_t lum;

    Serial.println("\r\nTSL2591 Sensor: ");

    lum = tsl2591.getFullLuminosity();
    tsl2591_ir = (uint16_t)(lum >> 16);
    tsl2591_full = (uint16_t)(lum & 0xFFFF);
    tsl2591_visible = tsl2591_full - tsl2591_ir;
    tsl2591_lux = tsl2591.calculateLux(tsl2591_full, tsl2591_ir);

    Serial.print(" Light IR : ");
    Serial.println(tsl2591_ir);

    Serial.print(" Light Full : ");
    Serial.println(tsl2591_full);

    Serial.print(" Light Visible : ");
    Serial.println(tsl2591_visible);

    Serial.print(" Light LUX : ");
    Serial.println(tsl2591_lux, 6);
}

void lis3dh_get_data(void )
{
    uint8_t val;
    int16_t axis_data;

    Serial.println("\r\nLIS3DH Sensor: ");

    lis3dh_read_reg(LIS3DH_STATUS_REG_ADDR, &val);

    if(val & 0x08)
    {
        lis3dh_read_reg(LIS3DH_OUT_X_L_ADDR, &val);
        axis_data = ((int16_t)val) & 0x00ff;
        lis3dh_read_reg(LIS3DH_OUT_X_H_ADDR, &val);
        axis_data |= ((((int16_t)val) << 8) & 0xff00);
        axis_data = (axis_data) >> 4;
        lis3dh_data_x = axis_data;

        lis3dh_read_reg(LIS3DH_OUT_Y_L_ADDR, &val);
        axis_data = ((int16_t)val) & 0x00ff;
        lis3dh_read_reg(LIS3DH_OUT_Y_H_ADDR, &val);
        axis_data |= ((((int16_t)val) << 8) & 0xff00);
        axis_data = (axis_data) >> 4;
        lis3dh_data_y = axis_data;

        lis3dh_read_reg(LIS3DH_OUT_Z_L_ADDR, &val);
        axis_data = ((int16_t)val) & 0x00ff;
        lis3dh_read_reg(LIS3DH_OUT_Z_H_ADDR, &val);
        axis_data |= ((((int16_t)val) << 8) & 0xff00);
        axis_data = (axis_data) >> 4;
        lis3dh_data_z = axis_data;

        Serial.print(" Motion X : ");
        Serial.println(lis3dh_data_x);

        Serial.print(" Motion Y : ");
        Serial.println(lis3dh_data_y);

        Serial.print(" Motion Z : ");
        Serial.println(lis3dh_data_z);
    }
    else
    {
        Serial.println(" Not Ready");
    }
}

void get_battery(void )
{
    uint16_t adc = 0;
    uint8_t i;

#if ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QBUTTON))
    pinMode(LOW_BAT_EN_PIN, OUTPUT);
    digitalWrite(LOW_BAT_EN_PIN, HIGH);
#else
    pinMode(LOW_BAT_EN_PIN, OUTPUT);
    digitalWrite(LOW_BAT_EN_PIN, LOW);
#endif

    delay(10);

    Serial.println("\r\nBattery: ");

    // Read ADC
    for(i = 0; i < ADC_BUF_MAX; i++)
    {
        adc += analogRead(BATTERY_PIN);
        delay(1);
    }

#if ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QBUTTON))
    pinMode(LOW_BAT_EN_PIN, OUTPUT);
    digitalWrite(LOW_BAT_EN_PIN, LOW);
#else
    digitalWrite(LOW_BAT_EN_PIN, HIGH);
    pinMode(K1_PIN, INPUT_PULLUP);
#endif

    adc /= 8;

    Serial.print(" ADC Code: ");
    Serial.println(adc);

    battery_mV = (uint16_t)((ADC_REFERENCE_VOLTAGE / ADC_FULL_SCALE) *(float)adc * ADC_DIVIDER_VALUE);

    if(battery_mV >= ADC_POWER_DOWN_VOLTAGE_TRESHOULD)
    {
        battery_percent = (uint16_t)((((float)(battery_mV - ADC_POWER_DOWN_VOLTAGE_TRESHOULD)) / (ADC_BATTERY_VOLTAGE_NORMAL - ADC_POWER_DOWN_VOLTAGE_TRESHOULD)) * 100.0);
        if(battery_percent > 100)
        {
            battery_percent = 100;
        }
    }
    else
    {
        battery_percent = 0;
    }

    Serial.print(" Battery, mV: ");
    Serial.println(battery_mV);
    Serial.print(" Battery, percent: ");
    Serial.println(battery_percent);
}

void sysled_blink(void )
{
    detachInterrupt(digitalPinToInterrupt(K1_SYSLED_PIN));

    pinMode(K1_SYSLED_PIN, OUTPUT);
    digitalWrite(K1_SYSLED_PIN, LOW);
    delay(500);
    digitalWrite(K1_SYSLED_PIN, HIGH);
    delay(5);

    pinMode(K1_SYSLED_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(K1_SYSLED_PIN), user_key_interrupt, FALLING);
    user_key_interrupt_status = false;
}

void ICACHE_RAM_ATTR user_key_interrupt(void )
{
    user_key_interrupt_status = true;
}

void ICACHE_RAM_ATTR xint_interrupt(void )
{
    xint_interrupt_status = true;
}

void ICACHE_RAM_ATTR xint2_interrupt(void )
{
    xint2_interrupt_status = true;
}

bool lis3dh_read_reg(uint8_t RegisterAddr, uint8_t * p_RegisterValue)
{
    uint8_t ret;

    Wire.beginTransmission(LIS3DH_ADDRESS_I2C);
    Wire.write((uint8_t)RegisterAddr);
    ret = Wire.endTransmission(false);
    if(ret != 0)
    {
        return false;
    }

    Wire.requestFrom(LIS3DH_ADDRESS_I2C, 1);

    if(Wire.available())
    {
        *(p_RegisterValue) = (uint8_t)Wire.read();

        return true;
    }
    else
    {
        return false;
    }
}

bool lis3dh_write_reg(uint8_t RegisterAddr, uint8_t RegisterValue)
{
    uint8_t ret;

    Wire.beginTransmission(LIS3DH_ADDRESS_I2C);
    Wire.write((uint8_t)RegisterAddr);

    Wire.write((uint8_t)RegisterValue);

    ret = Wire.endTransmission();
    if(ret != 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}
