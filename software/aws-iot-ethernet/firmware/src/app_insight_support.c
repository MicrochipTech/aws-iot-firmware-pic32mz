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

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "app_insight_support.h"
#include "stdlib.h"
#include "stdbool.h"
#include "string.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

int APP_ParseCommand(char * pReadBuffer)
{
    if(pReadBuffer == NULL)
        return COMMAND_BAD_POINTER;
    JSON_Value *rootValue = json_parse_string(pReadBuffer);
    if (json_value_get_type(rootValue) != JSONObject)
            return COMMAND_BAD_JSON;
    JSON_Object * tObject = json_value_get_object(rootValue);
        
    if(json_object_dotget_string(tObject, "message.command") != NULL)
    {
        if(strcmp(json_object_dotget_string(tObject, "message.command"), "hello") == 0)
        {
            return COMMAND_HELLO;
        }
        else if (strcmp(json_object_dotget_string(tObject, "message.command"), "configuration") == 0)
        {
            return COMMAND_CONFIGURE;
        }
        else if (strcmp(json_object_dotget_string(tObject, "message.command"), "debug_set") == 0)
        {
            return COMMAND_DEBUG_SET;
        }
        else
        {
            return COMMAND_INVALID;
        }
    }
    else
    {
        return COMMAND_INVALID;
    }
    return COMMAND_INVALID;
}

void BuildAckNackMessage(JSON_Object *rootObject, char * command, char * msg)
{
    json_object_dotset_string(rootObject, "message.command", command);
    json_object_dotset_string(rootObject, "message.ack_nack_message", msg);
}

int APP_CheckForMessage(char * pMessage)
{
    if(pMessage == NULL)
        return MESSAGE_BAD_POINTER;
    if(strchr(pMessage, DELIMITER))
    {
        return MESSAGE_DELIMITER;
    }
    else
    {
        return MESSAGE_NO_DELIMITER;
    }

}

void APP_BuildDebugString(char * messageBuffer, int sizeMessageBuffer, int command)
{
    if(messageBuffer == NULL)
        return;
    
    
}

const char* APP_ReturnDebugCodeToString(int debugCommand)
{
    switch(debugCommand)
    {
        case DEBUG_UPDATE:
            return "Debug update";
        case DEBUG_NVM_WRITE_SUCCESS:
            return "Configuration successfully written to NVM";   
        case DEBUG_NVM_WRITE_FAILED:
            return "Configuration failed to write to NVM, reset and try again";  
        case DEBUG_NO_IP_ADDRESS:
            return "Not getting an IP address, check connection";
        case DEBUG_SOCKET_CHANGE:
            return "Change socket state";    
        case DEBUG_MQTT_CHANGE:
            return "Change mqtt state";  
        case DEBUG_GOT_IP_ADDRESS:
            return "Got IP address";
        case DEBUG_ERROR_CLOSING_SOCKET:
            return "Error, closing socket and retrying";
        case DEBUG_FATAL_ERROR:
            return "Fatal Error, requires board reset";
            
    }
    return "Unknown debug command";
}

/* *****************************************************************************
 End of File
 */
