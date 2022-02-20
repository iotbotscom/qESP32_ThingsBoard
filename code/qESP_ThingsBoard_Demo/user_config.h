#ifndef USER_CONFIG_H
#define USER_CONFIG_H

/*---- Include files -------------------------------------------------------------------*/

/*---- Defines  ------------------------------------------------------------------------*/
// IOT Board options
#define QESP_IOT_BOARD_TYPE_QESP                1  // ESP8266 based qESP IOT board
#define QESP_IOT_BOARD_TYPE_QESP32              2  // ESP32 based qESP IOT board
#define QESP_IOT_BOARD_TYPE_QBUTTON             3  // ESP32 based qESP IOT board

// Project HW option : qESP32
#define QESP_IOT_BOARD_TYPE                     QESP_IOT_BOARD_TYPE_QESP32

// Project Sleep Mode Options
#define QESP_IOT_BOARD_SLEEP_MODE_ALWAYS_ON     0
#define QESP_IOT_BOARD_SLEEP_MODE_TIMER         1
#define QESP_IOT_BOARD_SLEEP_MODE_MOTION        2
#define QESP_IOT_BOARD_SLEEP_MODE_USER_KEY      4

// Always On
//#define QESP_IOT_BOARD_SLEEP_MODE_TYPE          QESP_IOT_BOARD_SLEEP_MODE_ALWAYS_ON
// Timer, User Button and Motion Wake Up
//#define QESP_IOT_BOARD_SLEEP_MODE_TYPE          (QESP_IOT_BOARD_SLEEP_MODE_TIMER | QESP_IOT_BOARD_SLEEP_MODE_MOTION | QESP_IOT_BOARD_SLEEP_MODE_USER_KEY)
// Motion Wake Up
#define QESP_IOT_BOARD_SLEEP_MODE_TYPE          (QESP_IOT_BOARD_SLEEP_MODE_MOTION)

/*---- Typedefs ------------------------------------------------------------------------*/

/*---- Variables -----------------------------------------------------------------------*/

/*---- Function prototypes -------------------------------------------------------------*/

/*---- Function declarations------------------------------------------------------------*/

#endif
