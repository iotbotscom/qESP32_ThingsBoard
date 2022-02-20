#ifndef HW_CONFIG_QESP32_H
#define HW_CONFIG_QESP32_H

/*---- Include files -------------------------------------------------------------------*/

/*---- Defines  ------------------------------------------------------------------------*/

// HW Version / Product Type
#define HW_PRODUCT_VERSION  "01.00"
#define HW_PRODUCT_TYPE     "qESP32"

// HW Pin defines
// I2C
#define I2C_SDA         23
#define I2C_SCL         22

// Motion Sensor Interrupts
#define XINT_PIN        32
#define XINT2_PIN       33

// External Interrupt
#define EXT_INT_PIN     39

// Light Sensor Interrupt
#define TSL_INT_PIN     36

// Battery Man
#define LOW_BAT_EN_PIN  27
#define BATTERY_PIN     35

// Audio / Speaker
#define AUDIO_PIN       25

// User Button
#define K1_PIN          0

// Indication
#define SYSLED_PIN      0

// K1 & LED Common Pin
#define K1_SYSLED_PIN   0

// Load Enable
#define LOAD_EN_PIN     14

// TPS61291DRV DC/DC On / Off
#define HIGH_PWR_ON_PIN 26

/*---- Typedefs ------------------------------------------------------------------------*/

/*---- Variables -----------------------------------------------------------------------*/

/*---- Function prototypes -------------------------------------------------------------*/

/*---- Function declarations------------------------------------------------------------*/

#endif

