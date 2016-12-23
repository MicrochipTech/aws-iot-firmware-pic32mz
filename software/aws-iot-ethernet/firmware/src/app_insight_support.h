/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app2.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#include "parson.h"

#ifndef _APP_INSIGHT_SUPPORT_H    /* Guard against multiple inclusion */
#define _APP_INSIGHT_SUPPORT_H

#define DELIMITER '\r'

enum ErrorCodes
{
    MESSAGE_DELIMITER,
    MESSAGE_NO_DELIMITER,
    MESSAGE_BAD_POINTER
};

enum CommandCodes
{
    COMMAND_BAD_POINTER,
    COMMAND_BAD_JSON,
    COMMAND_INVALID,
    COMMAND_HELLO,
    COMMAND_CONFIGURE,
    COMMAND_WIFI_CONFIGURE,
    COMMAND_DEBUG_MSG,
    COMMAND_DISCOVERY,
    COMMAND_ACK,
    COMMAND_NACK,
    COMMAND_DEBUG_SET
};

enum DebugCodes 
{
    DEBUG_UPDATE,
    DEBUG_NVM_WRITE_FAILED,
    DEBUG_NVM_WRITE_SUCCESS,
    DEBUG_NO_IP_ADDRESS,
    DEBUG_SOCKET_CHANGE,
    DEBUG_MQTT_CHANGE,
    DEBUG_GOT_IP_ADDRESS,
    DEBUG_ERROR_CLOSING_SOCKET,
    DEBUG_FATAL_ERROR
};

int APP_ParseCommand(char * pReadBuffer);
int APP_CheckForMessage(char * pMessage);
void BuildAckNackMessage(JSON_Object *rootObject, char * command, char * msg);
const char* APP_ReturnDebugCodeToString(int return_code);

#endif /* _APP_INSIGHT_SUPPORT_H */

/* *****************************************************************************
 End of File
 */
