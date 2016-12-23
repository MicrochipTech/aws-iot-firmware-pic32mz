/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app2.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app2.h"
#include "app_insight_support.h"
#include "parson.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
extern APP_DATA appData;

char APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];
char APP_MAKE_BUFFER_DMA_READY writeBuffer[APP_WRITE_BUFFER_SIZE];
char APP_MAKE_BUFFER_DMA_READY messageBuffer[APP_MESSAGE_BUFFER_SIZE];

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP2_DATA app2Data;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void * pData,
    uintptr_t userData
)
{
    APP2_DATA * app2DataObject;
    app2DataObject = (APP2_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(app2DataObject->deviceHandle,
                    &app2DataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(app2DataObject->deviceHandle,
                    &app2DataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            app2DataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            app2DataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(app2DataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            app2DataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *)pData)->breakDuration;
            
            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(app2DataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            //SYS_CONSOLE_MESSAGE("Read flag set\n");
            app2DataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(app2DataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            app2DataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context )
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    static bool led;
    
    switch ( event )
    {
        case USB_DEVICE_EVENT_SOF:
            app2Data.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:
            
            app2Data.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:    
            
            /* Set the USBID pin to output (LED control) after configuration is done */
            PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);
            
            (led) ? PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3) : PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);

            /* Check the configuration. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;
            if ( configuredEventData->configurationValue == 1)
            {
                /* Register the CDC Device application event handler here.
                 * Note how the app2Data object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t)&app2Data);

                /* Mark that the device is now configured */
                app2Data.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:
            
            led = PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);
            
            /* Set the USBID pin to input (floating) so that it will be configured as DEVICE */
            PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);
            
            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(app2Data.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(app2Data.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

bool APP_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(app2Data.isConfigured == false)
    {
        app2Data.state = APP2_STATE_WAIT_FOR_CONFIGURATION;
        app2Data.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        app2Data.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        app2Data.isReadComplete = false;
        app2Data.isWriteComplete = false;
        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP2_Initialize ( void )

  Remarks:
    See prototype in app2.h.
 */

void APP2_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app2Data.state = APP2_STATE_INIT;

    /* Device Layer Handle  */
    app2Data.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    app2Data.isConfigured = false;

    /* Initial get line coding state */
    app2Data.getLineCodingData.dwDTERate = 115200;
    app2Data.getLineCodingData.bParityType =  0;
    app2Data.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    app2Data.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    app2Data.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    app2Data.isReadComplete = true;

    /*Initialize the write complete flag*/
    app2Data.isWriteComplete = true;

    /* Reset the switch debounce counter */
    app2Data.switchDebounceTimer = 0;

    /* Reset other flags */
    app2Data.sofEventHasOccurred = false;

    /* Set up the read buffer */
    app2Data.readBuffer = &readBuffer[0];
    
    /* Set up the write buffer */
    app2Data.writeBuffer = &writeBuffer[0];
    
    /* Initialize the timers */
    app2Data.messageTimeout = 0;

    /* Setup debug message queue */
    app2Data.debugQueue = xQueueCreate( 20, sizeof(int) );
    if(app2Data.debugQueue == NULL) {
        ; // Handle this
    }
    xQueueReset(app2Data.debugQueue);
}


/******************************************************************************
  Function:
    void APP2_Tasks ( void )

  Remarks:
    See prototype in app2.h.
 */

void APP2_Tasks ( void )
{
/* Update the application state machine based
     * on the current state */

    switch(app2Data.state)
    {
        case APP2_STATE_INIT:

            /* Open the device layer */
            app2Data.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(app2Data.deviceHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(app2Data.deviceHandle, APP_USBDeviceEventHandler, 0);

                app2Data.state = APP2_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP2_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if(app2Data.isConfigured)
            {
                app2Data.state = APP2_STATE_SCHEDULE_READ;
            }
            break;
            
        case APP2_STATE_SCHEDULE_READ:
            if(APP_StateReset())
            {
                break;
            }
            
            /* If a read is complete, then schedule a read
            * else wait for the current read to complete */
            
            app2Data.state = APP2_STATE_WAIT_FOR_READ_COMPLETE;
            if(app2Data.isReadComplete == true)
            {
                app2Data.isReadComplete = false;
                app2Data.readTransferHandle =  USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
                
                USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                        &app2Data.readTransferHandle, app2Data.readBuffer,
                        APP_READ_BUFFER_SIZE);
                
                if(app2Data.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)
                {
                    app2Data.state = APP2_STATE_ERROR;
                    break;
                }
            }
            break;

        case APP2_STATE_WAIT_FOR_READ_COMPLETE:
            
            if(APP_StateReset())
            {
                break;
            }
            
            if(app2Data.isReadComplete == true)
            {            
                // Check for delimiter
                int res = APP_CheckForMessage(app2Data.readBuffer);
                if(res == MESSAGE_BAD_POINTER)
                {
                    // Do something
                    break;
                }
                else if(res == MESSAGE_NO_DELIMITER)
                {
                    strcat(messageBuffer, app2Data.readBuffer);
                    memset(app2Data.readBuffer, '\0', APP_READ_BUFFER_SIZE);
                    app2Data.state = APP2_STATE_SCHEDULE_READ;
                    break;
                }
 
                strcat(messageBuffer, app2Data.readBuffer);
                // Parse for the command being sent so we can handle it
                int command = APP_ParseCommand(messageBuffer);

                // Lets build our response message
                // First initialize the JSON value and object from parson library
                JSON_Value *rootValue = json_value_init_object();
                JSON_Object *rootObject = json_value_get_object(rootValue);
                char *serializedString = NULL;  
                    
                // Build response and handle the command
                switch(command)
                {
                    // If hello command, respond with discovery packet 
                    case COMMAND_HELLO:
                            json_object_dotset_string(rootObject, "message.command", "discovery");
                            json_object_dotset_string(rootObject, "message.discovery_object.title", APP_TITLE);
                            json_object_dotset_string(rootObject, "message.discovery_object.part_number", APP_PART_NUMBER);
                            json_object_dotset_string(rootObject, "message.discovery_object.mac_address", appData.macAddress);
                            json_object_dotset_string(rootObject, "message.discovery_object.firmware_version", APP_FIRMWARE_VERSION);
                            json_object_dotset_string(rootObject, "message.discovery_object.harmony_version", SYS_VERSION_STR);
                            json_object_dotset_boolean(rootObject, "message.discovery_object.is_commissioned", appData.isCommissioned);
                        break;
                        
                    // If configuration command, respond with ACK/NACK 
                    case COMMAND_CONFIGURE:
                    {
                        if(appData.isCommissioned == false)
                        {
                            JSON_Value *messageRoot = json_parse_string(messageBuffer);
                            if(json_value_get_type(messageRoot) != JSONObject)
                            {
                                BuildAckNackMessage(rootObject, "nack", "Invalid JSON Object");
                                break;
                            }
                            else
                            {
                                JSON_Object *messageObject = json_value_get_object(messageRoot);
                                if( json_object_dotget_string(messageObject, "message.configuration_object.aws_iot_endpoint_address") != NULL &&
                                    json_object_dotget_string(messageObject, "message.configuration_object.aws_certificate") != NULL &&
                                    json_object_dotget_string(messageObject, "message.configuration_object.aws_certificate_private_key") != NULL )
                                {
                                    json_object_dotset_string(rootObject, "message.command", "ack");
                                    json_object_dotset_string(rootObject, "message.ack_nack_message", "Writing commission parameters to flash");
                                    sprintf((char *)appData.host, json_object_dotget_string(messageObject, "message.configuration_object.aws_iot_endpoint_address"));
                                    sprintf((char *)appData.clientCert, json_object_dotget_string(messageObject, "message.configuration_object.aws_certificate"));
                                    sprintf((char *)appData.clientKey, json_object_dotget_string(messageObject, "message.configuration_object.aws_certificate_private_key"));
                                    appData.isCommissioned = true;
                                    appData.writeToNVM = true;
                                }
                                else
                                {
                                    BuildAckNackMessage(rootObject, "nack", "Invalid commission parameters");
                                    break;
                                }
                            } 
                        }
                        else
                        {
                            BuildAckNackMessage(rootObject, "nack", "Already commissioned");
                        }                    
                        break;
                    }
                    
                    case COMMAND_DEBUG_SET:
                    {
                        JSON_Value *messageRoot = json_parse_string(messageBuffer);
                        if(json_value_get_type(messageRoot) != JSONObject)
                        {
                            BuildAckNackMessage(rootObject, "nack", "Invalid JSON Object");
                            break;
                        }
                        JSON_Object *messageObject = json_value_get_object(messageRoot);
                        if(messageObject == NULL)
                        {
                            BuildAckNackMessage(rootObject, "nack", "NULL Pointer");
                            break;
                        }
                        BuildAckNackMessage(rootObject, "ack", "Received debug set");
                        int DebugMessage = DEBUG_UPDATE;
                        xQueueSendToFront(app2Data.debugQueue, &DebugMessage, 1);
                        if(json_object_dotget_boolean(messageObject, "message.debug_set_object.set"))
                            appData.debugSet = true;
                        else
                            appData.debugSet = false;
                        break;
                    }
                    
                    case COMMAND_BAD_JSON:
                            BuildAckNackMessage(rootObject, "nack", "Bad JSON");
                        break;
                    
                    case COMMAND_INVALID:
                            BuildAckNackMessage(rootObject, "nack", "Command Invalid");
                        break;
                        
                    default:
                            BuildAckNackMessage(rootObject, "nack", "Something went wrong");
                        break;
                }
                memset(app2Data.writeBuffer, '\0', APP_WRITE_BUFFER_SIZE);
                // With our response built, serialize the response into a string
                serializedString = json_serialize_to_string(rootValue);
                strcpy(app2Data.writeBuffer, serializedString);
                // Find length of string and add delimiter to end
                app2Data.writeBuffer[strlen(app2Data.writeBuffer)] = '\r';
                
                json_free_serialized_string(serializedString);  
                // Reset string buffers
                memset(app2Data.readBuffer, '\0', APP_READ_BUFFER_SIZE);
                memset(messageBuffer, '\0', APP_MESSAGE_BUFFER_SIZE);
                app2Data.state = APP2_STATE_SCHEDULE_WRITE;
            }
            else
            {
                app2Data.state = APP2_STATE_WAIT_FOR_READ_COMPLETE;
                if( uxQueueMessagesWaiting( app2Data.debugQueue ) > 0 )
                {
                    int debugMessageNumber;
                    xQueueReceive( app2Data.debugQueue, &debugMessageNumber, 1 );
                    sprintf(app2Data.writeBuffer,                       
                    "{"                                                  
                         "\"message\":{"                                 
                            "\"command\":\"debug\","
                                "\"debug_object\":{"
                                    "\"board_ip_address\":\"%d.%d.%d.%d\","
                                    "\"aws_iot_endpoint\":\"%s\","
                                    "\"mac_address\":\"%s\","
                                    "\"socket_connected\":%s,"
                                    "\"mqtt_connected\":%s,"
                                    "\"raw_message\":\"%s\""
                    "}}}\r",
                    appData.board_ipAddr.v4Add.v[0],
                    appData.board_ipAddr.v4Add.v[1],
                    appData.board_ipAddr.v4Add.v[2],
                    appData.board_ipAddr.v4Add.v[3],
                    appData.host,
                    appData.macAddress,
                    (appData.socket_connected ? "true" : "false"),
                    (appData.mqtt_connected ? "true" : "false"),
                    APP_ReturnDebugCodeToString(debugMessageNumber));
                    app2Data.state = APP2_STATE_SCHEDULE_DEBUG_WRITE;
                }
            }
            break;
            
                        
        case APP2_STATE_SCHEDULE_WRITE:

            if(APP_StateReset())
            {
                break;
            }

            app2Data.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            app2Data.isWriteComplete = false;
            app2Data.state = APP2_STATE_WAIT_FOR_WRITE_COMPLETE;

            USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                &app2Data.writeTransferHandle, app2Data.writeBuffer, strlen(app2Data.writeBuffer),
                USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);

            break;
            
        case APP2_STATE_WAIT_FOR_WRITE_COMPLETE:

            if(APP_StateReset())
            {
                break;
            }

            /* Check if message sent.  The isWriteComplete
             * flag gets updated in the CDC event handler */

            if(app2Data.isWriteComplete == true)
            {
                app2Data.state = APP2_STATE_SCHEDULE_READ;
            }
            break;
            
        case APP2_STATE_SCHEDULE_DEBUG_WRITE:
            if(APP_StateReset())
            {
                break;
            }

            app2Data.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            app2Data.isWriteComplete = false;
            app2Data.state = APP2_STATE_WAIT_FOR_DEBUG_WRITE_COMPLETE;

            USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                &app2Data.writeTransferHandle, app2Data.writeBuffer, strlen(app2Data.writeBuffer),
                USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);

            break;
            
        case APP2_STATE_WAIT_FOR_DEBUG_WRITE_COMPLETE:

            if(APP_StateReset())
            {
                break;
            }

            /* Check if message sent.  The isWriteComplete
             * flag gets updated in the CDC event handler */

            if(app2Data.isWriteComplete == true)
            {
                app2Data.state = APP2_STATE_WAIT_FOR_READ_COMPLETE;
            }
            break;

        case APP2_STATE_ERROR:
            break;
        default:
            break;
    }
}

 

/*******************************************************************************
 End of File
 */
