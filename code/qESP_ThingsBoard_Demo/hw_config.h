#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/*---- Include files -------------------------------------------------------------------*/

#include "user_config.h"

#if ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QESP))
 #include "hw_config_qesp.h"
#elif ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QESP32))
 #include "hw_config_qesp32.h"
#elif ((defined QESP_IOT_BOARD_TYPE) && (QESP_IOT_BOARD_TYPE == QESP_IOT_BOARD_TYPE_QBUTTON))
 #include "hw_config_qbutton.h"
#else
    Serial.print("\r\n **** qESP32 IOT Board ThingsBoard Demo **** \r\n");
#endif

/*---- Defines  ------------------------------------------------------------------------*/

/*---- Typedefs ------------------------------------------------------------------------*/

/*---- Variables -----------------------------------------------------------------------*/

/*---- Function prototypes -------------------------------------------------------------*/

/*---- Function declarations------------------------------------------------------------*/

#endif

