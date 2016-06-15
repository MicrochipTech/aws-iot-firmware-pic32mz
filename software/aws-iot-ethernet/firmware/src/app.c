/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

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

#include "app.h"
#include "app1.h"
#include "parson.h"
#include "bsp_config.h"
#include "app_nvm_support.h"
#include "wolfmqttsdk/wolfmqtt/mqtt_client.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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

APP_DATA appData;
extern APP1_DATA app1Data;

#define APP_HARDWARE  "iot_ethernet_dm990004"
#define APP_FIRMWARE_VERSION "1.1.0"

char topic_awsUpdate[128];
char topic_awsUpdateDelta[128];

#define MQTT_DEFAULT_CMD_TIMEOUT_MS 10000
#define MAX_BUFFER_SIZE 1024
#define MAX_PACKET_ID 65536
#define KEEP_ALIVE 900


byte txBuffer[MAX_BUFFER_SIZE];
byte rxBuffer[MAX_BUFFER_SIZE];

static int mPacketIdLast;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

// WolfMQTT Callbacks for network connectivity
int APP_tcpipConnect_cb(void *context, const char* host, word16 port, int timeout_ms)
{
    uint32_t timeout = 0;
    timeout = SYS_TMR_TickCountGet();
    SYS_CONSOLE_PRINT("App:  DNS:   Resolving host '%s'\r\n", &appData.host);
    TCPIP_DNS_RESULT dnsResult;
    
    dnsResult = TCPIP_DNS_Resolve((const char *)appData.host, TCPIP_DNS_TYPE_A);
    if(dnsResult < 0)
    {
        SYS_CONSOLE_MESSAGE("App:  DNS:  Failed to begin\r\n");
        return APP_CODE_ERROR_FAILED_TO_BEGIN_DNS_RESOLUTION;
    }

    while((dnsResult = TCPIP_DNS_IsResolved((const char *)appData.host, &appData.host_ipv4, IP_ADDRESS_TYPE_IPV4)) == TCPIP_DNS_RES_PENDING)
    {
        if(APP_TIMER_Expired_ms(&timeout, timeout_ms))
        {
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }
    if(dnsResult != (TCPIP_DNS_RES_OK))
    {
        SYS_CONSOLE_PRINT("App:  DNS:  Resolution failed - Aborting\r\n");
        return APP_CODE_ERROR_DNS_FAILED;
    } 
    else if(dnsResult == TCPIP_DNS_RES_OK)
    {
       SYS_CONSOLE_PRINT("App:  DNS:  Resolved IPv4 Address: %d.%d.%d.%d for host '%s'\r\n", 
            appData.host_ipv4.v4Add.v[0],appData.host_ipv4.v4Add.v[1],appData.host_ipv4.v4Add.v[2],
            appData.host_ipv4.v4Add.v[3],appData.host);
    }  
    SYS_CONSOLE_PRINT("App:  TCPIP:  Opening socket to '%d.%d.%d.%d:%d'\r\n", 
        appData.host_ipv4.v4Add.v[0], appData.host_ipv4.v4Add.v[1], appData.host_ipv4.v4Add.v[2],
        appData.host_ipv4.v4Add.v[3], appData.port);
    
    uint32_t timeSocketbefore = SYS_TMR_TickCountGet();
    appData.socket = NET_PRES_SocketOpen(0, NET_PRES_SKT_ENCRYPTED_STREAM_CLIENT, IP_ADDRESS_TYPE_IPV4, (NET_PRES_SKT_PORT_T)port, (NET_PRES_ADDRESS *)&appData.host_ipv4, (NET_PRES_SKT_ERROR_T*)&appData.error);
    NET_PRES_SocketWasReset(appData.socket);
    
    if(appData.socket == INVALID_SOCKET)
    {
        SYS_CONSOLE_MESSAGE("App:  TCPIP:  Invalid socket error\r\n");
        NET_PRES_SocketClose(appData.socket);
        return APP_CODE_ERROR_INVALID_SOCKET;
    }
 
    while(!NET_PRES_SKT_IsConnected(appData.socket))
    {
        if(APP_TIMER_Expired_ms(&timeout, timeout_ms))
        {
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }
    while(NET_PRES_SKT_IsNegotiatingEncryption(appData.socket))
    {
        if(APP_TIMER_Expired_ms(&timeout, timeout_ms))
        {
            return APP_CODE_ERROR_CMD_TIMEOUT;
        }
    }
        
    if (!NET_PRES_SKT_IsSecure(appData.socket))
    {
        SYS_CONSOLE_MESSAGE("App:  TCPIP:  SSL failed to negotiate\r\n");
        NET_PRES_SocketClose(appData.socket);
        return APP_CODE_ERROR_FAILED_SSL_NEGOTIATION;
    }
    uint32_t timeSocketafter = SYS_TMR_TickCountGet();
    SYS_CONSOLE_PRINT("App:  Socket Opened - Time to open %d ticks\r\n", timeSocketafter-timeSocketbefore);
    
    return 0; //Success
}

int APP_tcpipWrite_cb(void *context, const byte* buf, int buf_len, int timeout_ms)
{
    int ret = 0;
    uint32_t timeout;
    
    APP_TIMER_Set(&timeout);
    //wait for data to be read, or error, or timeout
    while(NET_PRES_SocketWriteIsReady(appData.socket, buf_len, 0) == 0)
    {
        if(NET_PRES_SocketWasReset(appData.socket))
        {
            ret = APP_CODE_ERROR_SSL_FATAL;
            return ret;
        }
        if(APP_TIMER_Expired_ms(&timeout, (uint32_t)timeout_ms))
        {
            ret = APP_CODE_ERROR_CMD_TIMEOUT;
            return ret;
        }
    }
    ret = NET_PRES_SocketWrite(appData.socket, (uint8_t*)buf, buf_len);
    return ret;
}

int APP_tcpipRead_cb(void *context, byte* buf, int buf_len, int timeout_ms)
{
    int ret = 0;
    uint32_t timeout;
    
    APP_TIMER_Set(&timeout);
    // Wait for data to be read, or error, or timeout
    while(NET_PRES_SocketReadIsReady(appData.socket) == 0)
    {
        if(NET_PRES_SocketWasReset(appData.socket))
        {
            ret = APP_CODE_ERROR_SSL_FATAL;
            return ret;
        }
        if(APP_TIMER_Expired_ms(&timeout, (uint32_t)timeout_ms))
        {
            ret = APP_CODE_ERROR_CMD_TIMEOUT;
            return ret;
        }
    }
    ret = NET_PRES_SocketRead(appData.socket, (uint8_t*)buf, buf_len);
    return ret;
}

int APP_tcpipDisconnect_cb(void *context)
{
    int ret = 0;
    NET_PRES_SKT_Close(appData.socket);
    appData.state = APP_TCPIP_MQTT_NET_CONNECT;
    return ret;
}

static word16 mqttclient_get_packetid(void)
{
    mPacketIdLast = (mPacketIdLast >= MAX_PACKET_ID) ?
        1 : mPacketIdLast + 1;
    return (word16)mPacketIdLast;
}

const char* APP_Switch_Publish_Helper(BSP_SWITCH_ENUM sw)
{
    switch (sw)
    {
        case BSP_SWITCH_1:
            return "state.reported.button1";
        case BSP_SWITCH_2:
            return "state.reported.button2";
        case BSP_SWITCH_3:
            return "state.reported.button3";
        case BSP_SWITCH_4:
            return "state.reported.button4";
        default:
            return 0;
            break;
    }
}

int mqttclient_message_cb(MqttClient *client, MqttMessage *msg, byte msg_new, byte msg_done)
{
    char payload[MAX_BUFFER_SIZE];
    memcpy(payload, msg->buffer, msg->total_len);
    payload[msg->total_len] = '\0';
    SYS_CONSOLE_PRINT("\r\nApp:  MQTT.Message Received: %s -- Topic %*.*s\r\n\r\n", payload, msg->topic_name_len, msg->topic_name_len, msg->topic_name);
    
    appData.lightShowVal = BSP_LED_RX;
    xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);
    
    // If the topic matches our AWS IoT delta topic
    // "$aws/things/<macaddress>/shadow/update/delta"
    if(strncmp(topic_awsUpdateDelta, msg->topic_name, strlen(topic_awsUpdateDelta)) == 0)
    {
        JSON_Value *root_value = json_parse_string(payload);
        if (json_value_get_type(root_value) != JSONObject)
            return -1;
        JSON_Object * tObject = json_value_get_object(root_value);
        
        if(json_object_dotget_string(tObject, "state.led1") != NULL)
        {
            appData.led1 = true;
            if(strcmp(json_object_dotget_string(tObject, "state.led1"), "on") == 0)
            {
                BSP_LEDOn(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                appData.led1val = true;
            }
            else if (strcmp(json_object_dotget_string(tObject, "state.led1"), "off") == 0)
            {
                BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                appData.led1val = false;
            }
        }
        if(json_object_dotget_string(tObject, "state.led2") != NULL)
        {
            appData.led2 = true;
            if(strcmp(json_object_dotget_string(tObject, "state.led2"), "on") == 0)
            {
                BSP_LEDOn(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                appData.led2val = true;
            }
            else if (strcmp(json_object_dotget_string(tObject, "state.led2"), "off") == 0)
            {
                BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                appData.led2val = false;
            }
        }
        if(json_object_dotget_string(tObject, "state.led3") != NULL)
        {
            appData.led3 = true;
            if(strcmp(json_object_dotget_string(tObject, "state.led3"), "on") == 0)
            {
                BSP_LEDOn(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                appData.led3val = true;
            }
            else if (strcmp(json_object_dotget_string(tObject, "state.led3"), "off") == 0)
            {
                BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                appData.led3val = false;
            }
        }
        if(json_object_dotget_string(tObject, "state.led4") != NULL)
        {
            appData.led4 = true;
            if(strcmp(json_object_dotget_string(tObject, "state.led4"), "on") == 0)
            {
                BSP_LEDOn(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                appData.led4val = true;
            }
            else if (strcmp(json_object_dotget_string(tObject, "state.led4"), "off") == 0)
            {
                BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                appData.led4val = false;
            }
        }
        
        //Got LED Values now we send our reported LED values
        JSON_Value *root_value_publish = json_value_init_object();
        JSON_Object *root_object_publish = json_value_get_object(root_value_publish);
        char *serialized_string = NULL;
        
        if(appData.led1)
        {
            json_object_dotset_string(root_object_publish, "state.reported.led1", appData.led1val ? "on" : "off");
        }
        if(appData.led2)
        {
            json_object_dotset_string(root_object_publish, "state.reported.led2", appData.led2val ? "on" : "off");
        }
        if(appData.led3)
        {
            json_object_dotset_string(root_object_publish, "state.reported.led3", appData.led3val ? "on" : "off");
        }
        if(appData.led4)
        {
            json_object_dotset_string(root_object_publish, "state.reported.led4", appData.led4val ? "on" : "off");
        }
        appData.led1 = appData.led2 = appData.led3 = appData.led4 = false;
        
        // We build our PUBLISH payload
        char reportedPayload[1024];
        
        serialized_string = json_serialize_to_string(root_value_publish);
        strcpy(reportedPayload, serialized_string);
        json_free_serialized_string(serialized_string);
        
        /* Publish Topic */
        MqttPublish publish;
        int rc;
        XMEMSET(&publish, 0, sizeof(MqttPublish));
        publish.retain = 0;
        publish.qos = 0;
        publish.duplicate = 0;
        publish.topic_name = topic_awsUpdate;
        publish.packet_id = mqttclient_get_packetid();
        publish.buffer = (byte *)reportedPayload;
        publish.total_len = strlen((char *)publish.buffer);
        rc = MqttClient_Publish(&appData.myClient, &publish);
        SYS_CONSOLE_PRINT("App:  MQTT.Publish: Topic %s, %s (%d)\r\n    Payload:  %s\r\n",
            publish.topic_name, MqttClient_ReturnCodeToString(rc), rc, publish.buffer);
        if (rc != MQTT_CODE_SUCCESS) {
            while(1);
        }
        appData.lightShowVal = BSP_LED_TX;
        xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);
    
        json_value_free(root_value);
        json_value_free(root_value_publish);
    }   
    return 0;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


bool APP_TIMER_Expired(uint32_t * timer, uint32_t seconds)
{
    if((SYS_TMR_TickCountGet() - *timer) > (seconds * 1000))
    {
        return true;
    }
    else
    {
        return false;
    }
    return false;
}

bool APP_TIMER_Expired_ms(uint32_t * timer, uint32_t mseconds)
{
    if((SYS_TMR_TickCountGet() - *timer) > (mseconds)) 
    {
        return true;
    }
    else
    {
        return false;
    }
    return false;
}

bool APP_TIMER_Set(uint32_t * timer)
{
    *timer = SYS_TMR_TickCountGet();
    return true;
}

const char* APP_ReturnCodeToString(int return_code)
{
    switch(return_code)
    {
        case APP_CODE_SUCCESS:
            return "Success";
        case APP_CODE_ERROR_BAD_ARG:
            return "Error (Bad argument)";
        case APP_CODE_ERROR_OUT_OF_BUFFER:
            return "Error (Out of buffer)";
        case APP_CODE_ERROR_SSL_FATAL:
            return "Error (SSL Fatal)";
        case APP_CODE_ERROR_INVALID_SOCKET:
            return "Error (Invalid Socket)";
        case APP_CODE_ERROR_FAILED_TO_BEGIN_DNS_RESOLUTION:
            return "Error (Failed to Begin DNS)";
        case APP_CODE_ERROR_DNS_FAILED:
            return "Error (DNS Failed)";
        case APP_CODE_ERROR_FAILED_SSL_NEGOTIATION:
            return "Error (Failed SSL Negotiation)";
        case APP_CODE_ERROR_TIMEOUT:
            return "Error (Timeout)";
        case APP_CODE_ERROR_CMD_TIMEOUT:
            return "Error (Command Timeout)";
    }
    return "Unknown";
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    memset(appData.host, '\0', sizeof(appData.host));
    appData.port = AWS_IOT_PORT;
    
    // Initialize MQTT net callbacks
    appData.myNet.connect = APP_tcpipConnect_cb;
    appData.myNet.disconnect = APP_tcpipDisconnect_cb;
    appData.myNet.read = APP_tcpipRead_cb;
    appData.myNet.write = APP_tcpipWrite_cb;
    
    // Init LED publish bools
    appData.led1 = false;
    appData.led2 = false;
    appData.led3 = false;
    appData.led4 = false;
    
    appData.led1val = false;
    appData.led2val = false;
    appData.led3val = false;
    appData.led4val = false;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{   
    static int validConfig = 0;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            if (appInitialized)
            {
                SYS_CONSOLE_MESSAGE("App:  Initialized\r\n");
                appData.state = APP_NVM_MOUNT_DISK;
            }
            break;
        }

        // Mount the file system where the webpages are loaded
        case APP_NVM_MOUNT_DISK:
        {
            if(SYS_FS_Mount(SYS_FS_NVM_VOL, LOCAL_WEBSITE_PATH_FS, MPFS2, 0, NULL) == 0)
            {
                SYS_CONSOLE_PRINT("App:  The %s File System is mounted.\r\n", SYS_FS_MPFS_STRING);
                appData.state = APP_NVM_ERASE_CONFIGURATION;
            }
            else
            {   // Timeout 5 seconds
                if(APP_TIMER_Expired(&appData.genericUseTimer, 5))
                {
                    SYS_CONSOLE_PRINT("App:  The %s File System failed to mount.  Critical Error, reset board\r\n", SYS_FS_MPFS_STRING);
                    appData.lightShowVal = BSP_LED_NVM_FAILED_MOUNT;
                    xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);  
                    while(1);
                }
            }
            break;
        }    
        
        // If user presses switch 2 and 3 on power up, the configuration will be erased
        case APP_NVM_ERASE_CONFIGURATION:
        {
            if((BSP_SWITCH_StateGet(BSP_SWITCH_3_CHANNEL, BSP_SWITCH_3_PORT) == BSP_SWITCH_STATE_ASSERTED) 
             && (BSP_SWITCH_StateGet(BSP_SWITCH_2_CHANNEL, BSP_SWITCH_2_PORT) == BSP_SWITCH_STATE_ASSERTED))
            {
                memset(appData.host, 0, sizeof(appData.host));
                APP_NVM_Write(NVM_HOST_ADDRESS_SPACE, appData.host);
                APP_NVM_Erase(NVM_CLIENT_CERTIFICATE_SPACE);
                APP_NVM_Erase(NVM_CLIENT_KEY_SPACE);
                SYS_CONSOLE_MESSAGE("***************************************\r\n"
                                    "App:  Erasing configuration!\r\n"
                                    "***************************************\r\n");
                appData.state = APP_TCPIP_WAIT_INIT;
                break;
            }
            appData.state = APP_NVM_LOAD_CONFIGURATION;
            break;
        }
        
        // Load the configuration stored in NVM on powerup
        case APP_NVM_LOAD_CONFIGURATION:
        {
            SYS_CONSOLE_MESSAGE("App:  Loading AWS IoT Endpoint Address and AWS Certificate/Certificate Private Key\r\n");
            APP_NVM_Read(NVM_HOST_ADDRESS_SPACE, appData.host, sizeof(appData.host));
            APP_NVM_Read(NVM_CLIENT_CERTIFICATE_SPACE, appData.clientCert, sizeof(appData.clientCert));
            APP_NVM_Read(NVM_CLIENT_KEY_SPACE, appData.clientKey, sizeof(appData.clientKey));
            if(appData.host[0] != '\0')
            {   // Set this flag so we know we loaded a valid config from NVM
                validConfig = 1;
            }
            appData.state = APP_TCPIP_WAIT_INIT;
            break;
        }
        
        // Wait for the TCPIP stack to initialize, store the boards MAC address and initialize mDNS service
        case APP_TCPIP_WAIT_INIT:
        {
            SYS_STATUS          tcpipStat;
            TCPIP_NET_HANDLE    netH;
            int                 i, nNets;
    
            tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(tcpipStat < 0) 
            {   // some error occurred
                break;
            }
            else if(tcpipStat == SYS_STATUS_READY) 
            {
                // now that the stack is ready we can check the
                // available interfaces
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                for(i = 0; i < nNets; i++) 
                {
                    netH = TCPIP_STACK_IndexToNet(i);
                    TCPIP_STACK_NetNameGet(netH);
                    TCPIP_STACK_NetBIOSName(netH);
             
                    // Retrieve MAC Address for UUID
                    TCPIP_NET_HANDLE netH = TCPIP_STACK_NetHandleGet("PIC32INT");
                    TCPIP_MAC_ADDR* pAdd = 0;
                    pAdd = (TCPIP_MAC_ADDR *)TCPIP_STACK_NetAddressMac(netH);
                    
                    // Store UUID for application
                    appData.macAddress.v[5] = pAdd->v[5];
                    appData.macAddress.v[4] = pAdd->v[4];
                    appData.macAddress.v[3] = pAdd->v[3];
                    appData.macAddress.v[2] = pAdd->v[2];
                    appData.macAddress.v[1] = pAdd->v[1];
                    appData.macAddress.v[0] = pAdd->v[0];
                    
                    // Convert to string
                    sprintf(appData.uuid, "%02x%02x%02x%02x%02x%02x",
                            appData.macAddress.v[0], appData.macAddress.v[1], appData.macAddress.v[2],
                            appData.macAddress.v[3], appData.macAddress.v[4], appData.macAddress.v[5]); 
                    
                    char mDNSServiceName[16]; // base name of the service Must not exceed 16 bytes long
                    strcpy(mDNSServiceName, &appData.uuid[6]); //Copy over UUID last 6 characters, 
                    strcat(mDNSServiceName, "_IoT-E");
                    SYS_CONSOLE_PRINT("App:  Registering mDNS service as '%s'\r\n", mDNSServiceName);
                    // mDNS name will be xxxxxx_IoT-E where "xxxxxx" is the last three bytes of MAC address
                    mDNSServiceName[sizeof(mDNSServiceName) - 2] = '1' + i;
                    TCPIP_MDNS_ServiceRegister( netH
                            , mDNSServiceName                     // name of the service
                            ,"_http._tcp.local"                   // type of the service
                            ,80                                   // TCP or UDP port, at which this service is available
                            ,((const uint8_t *)"path=/index.htm") // TXT info
                            ,1                                    // auto rename the service when if needed
                            ,NULL                                 // no callback function
                            ,NULL);      
                }
                
                // Here we build our Update and Delta topic strings using the boards unique MAC address
                sprintf(topic_awsUpdateDelta, "$aws/things/%02x%02x%02x%02x%02x%02x/shadow/update/delta", 
                    appData.macAddress.v[0], appData.macAddress.v[1], appData.macAddress.v[2],
                    appData.macAddress.v[3], appData.macAddress.v[4], appData.macAddress.v[5]);
                sprintf(topic_awsUpdate, "$aws/things/%02x%02x%02x%02x%02x%02x/shadow/update", 
                    appData.macAddress.v[0], appData.macAddress.v[1], appData.macAddress.v[2],
                    appData.macAddress.v[3], appData.macAddress.v[4], appData.macAddress.v[5]);
                APP_TIMER_Set(&appData.genericUseTimer);
                appData.state = APP_TCPIP_WAIT_FOR_IP;
            }
            break;
        }

        case APP_TCPIP_WAIT_FOR_IP:
        {
            IPV4_ADDR           ipAddr;
            TCPIP_NET_HANDLE    netH;
            int i, nNets;
            
            if(APP_TIMER_Expired(&appData.genericUseTimer, 5))
            {
                SYS_CONSOLE_MESSAGE("App:  Not getting IP Addr, check connections.  Retrying...\r\n");
                APP_TIMER_Set(&appData.genericUseTimer);
            }
                
            nNets = TCPIP_STACK_NumberOfNetworksGet();
            for (i = 0; i < nNets; i++)
            {
                netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                if( 0 != ipAddr.Val)
                {
                    if (ipAddr.v[0] != 0 && ipAddr.v[0] != 169) // Wait for a Valid IP
                    {
                        uint32_t lightShowVal = BSP_LED_EASY_CONFIGURATION;
                        xQueueSendToFront(app1Data.lightShowQueue, &lightShowVal, 1);
                        SYS_CONSOLE_PRINT("App:  Board online.  mDNS online. IP addr %d.%d.%d.%d online.  All systems nominal.\r\n",
                        ipAddr.v[0],ipAddr.v[1],ipAddr.v[2],ipAddr.v[3]);
                        SYS_CONSOLE_PRINT("App:  AWS Thing Name (MAC Address) '%s'\r\n", appData.uuid);
                        SYS_CONSOLE_MESSAGE("App:  Waiting for configuration...\r\n");
                        appData.state = APP_TCPIP_WAIT_CONFIGURATION;
                    }
                }
            }
            break;
        }
        
        case APP_TCPIP_WAIT_CONFIGURATION:
        {
            // We check if "host" is null, if it has a value, we assume we have a configuration
            if(appData.host[0] == '\0')
            {    
                break;
            }
            else
            {   // If validConfig flag is set, then we know the config came from reading NVM so we can skip this step
                if(validConfig == 0)
                {    
                    SYS_CONSOLE_MESSAGE("App:  Received configuration from webpage, writing to NVM...\r\n");
                    if(APP_NVM_Write(NVM_HOST_ADDRESS_SPACE, appData.host) &&
                        APP_NVM_Write(NVM_CLIENT_CERTIFICATE_SPACE, appData.clientCert) &&
                        APP_NVM_Write(NVM_CLIENT_KEY_SPACE, appData.clientKey))
                    {
                        SYS_CONSOLE_MESSAGE("App:  Writing configuration to NVM - success\r\n");
                    } 
                    else
                    {
                        SYS_CONSOLE_MESSAGE("App:  Writing configuration to NVM - failed\r\n");
                        while(1);
                    }
                    SYS_CONSOLE_PRINT("App:  Configured AWS IoT Endpoint Address '%s'\r\n", appData.host);
                } 
                else if(validConfig)
                {
                    SYS_CONSOLE_PRINT("App:  Found configuration - AWS IoT Endpoint Address '%s'\r\n", appData.host);
                }
                appData.lightShowVal = BSP_LED_INTIAL_CONNECT;
                xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);  
                TCPIP_NET_HANDLE    netH;
                int i, nNets;

                //Disable ZeroConf and HTTP server since we have server
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                for (i = 0; i < nNets; i++)
                {
                    netH = TCPIP_STACK_IndexToNet(i);
                    TCPIP_ZCLL_Disable(netH);
                }
            }
            appData.state = APP_TCPIP_MQTT_INIT;
            break;
        }
        

                
        case APP_TCPIP_MQTT_INIT:
        {
            SYS_CONSOLE_MESSAGE("App:  Beginning MQTT Client application\r\n");
            int rc = MqttClient_Init(&appData.myClient, &appData.myNet, mqttclient_message_cb, txBuffer, MAX_BUFFER_SIZE, rxBuffer, MAX_BUFFER_SIZE, MQTT_DEFAULT_CMD_TIMEOUT_MS);
            SYS_CONSOLE_PRINT("App:  MQTT.Client_Init: %s (%d)\r\n", MqttClient_ReturnCodeToString(rc), rc);
            if(rc != MQTT_CODE_SUCCESS)
            {
                SYS_CONSOLE_MESSAGE("App:  MQTT.Client_Init: Failed (catastrophic)\r\n");
                while(1);
            }
            APP_TIMER_Set(&appData.genericUseTimer);
            appData.state = APP_TCPIP_MQTT_NET_CONNECT;
            break;
        }
        
        case APP_TCPIP_MQTT_NET_CONNECT:
        {
            SYS_CONSOLE_MESSAGE("App:  MQTT.Net_Connect\r\n");
            int rc = MqttClient_NetConnect(&appData.myClient, (const char *)&appData.host, AWS_IOT_PORT, MQTT_DEFAULT_CMD_TIMEOUT_MS, NULL, NULL);
            SYS_CONSOLE_PRINT("App:  MQTT.Net_Connect: %s (%d)\r\n", MqttClient_ReturnCodeToString(rc), rc);
            if(rc != MQTT_CODE_SUCCESS)
            {
                SYS_CONSOLE_PRINT("App:  %s (%d)\r\n", APP_ReturnCodeToString(rc), rc);
                SYS_CONSOLE_PRINT("App:  Closing Socket %d\r\n\r\n", appData.socket);
                NET_PRES_SocketClose(appData.socket);
                appData.lightShowVal = BSP_LED_SERVER_CONNECT_FAILED;
                xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);  
                while(!APP_TIMER_Expired(&appData.genericUseTimer, 5));
                APP_TIMER_Set(&appData.genericUseTimer);
                break;
            }
            appData.lightShowVal = BSP_LED_ALL_GOOD;
            xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);  
            appData.state = APP_TCPIP_MQTT_PROTOCOL_CONNECT;
            break;
        }
        
        case APP_TCPIP_MQTT_PROTOCOL_CONNECT:
        {
            MqttConnect connect;
            MqttMessage lwt_msg;
            XMEMSET(&connect, 0, sizeof(MqttConnect));
            connect.keep_alive_sec = KEEP_ALIVE;
            connect.clean_session = 1;
            char clientIdString[13];
            sprintf(clientIdString, appData.uuid, "%02x%02x%02x%02x%02x%02x\0", 
                    appData.macAddress.v[0], appData.macAddress.v[1], appData.macAddress.v[2],
                    appData.macAddress.v[3], appData.macAddress.v[4], appData.macAddress.v[5]);
            connect.client_id = clientIdString;
            XMEMSET(&lwt_msg, 0, sizeof(lwt_msg));
            connect.lwt_msg = &lwt_msg;
            connect.enable_lwt = 0;
            
            /* Send Connect and wait for Connect Ack */
            int rc = MqttClient_Connect(&appData.myClient, &connect);
            SYS_CONSOLE_PRINT("App:  MQTT.Client_Connect: %s (%d)\r\n", MqttClient_ReturnCodeToString(rc), rc);
            if(rc != MQTT_CODE_SUCCESS)
            {
                SYS_CONSOLE_MESSAGE("App:  MQTT.Client_Connect: failed\r\n");
                APP_TIMER_Set(&appData.genericUseTimer);
                while(!APP_TIMER_Expired(&appData.genericUseTimer, 5));   
                appData.state = APP_TCPIP_ERROR;
                break;
            }
                /* Validate Connect Ack info */
            SYS_CONSOLE_PRINT("App:  MQTT.Connect Ack: Return Code %u, Session Present %d, keep alive: %d\r\n",
            connect.ack.return_code,
            (connect.ack.flags & MQTT_CONNECT_ACK_FLAG_SESSION_PRESENT) ?
                1 : 0, connect.keep_alive_sec);
            APP_TIMER_Set(&appData.mqttKeepAlive);
            appData.state = APP_TCPIP_MQTT_SUBSCRIBE;
            break;
        }
                
        case APP_TCPIP_MQTT_SUBSCRIBE:
        {
            MqttSubscribe subscribe;
            MqttTopic topics[1], *topic;
            MqttPublish publish;
            int i, rc;

            /* Build list of topics */
            topics[0].topic_filter = topic_awsUpdateDelta;
            topics[0].qos = 0;
            
            /* Subscribe Topic */
            XMEMSET(&subscribe, 0, sizeof(MqttSubscribe));
            subscribe.packet_id = mqttclient_get_packetid();
            subscribe.topic_count = sizeof(topics)/sizeof(MqttTopic);
            subscribe.topics = topics;
            rc = MqttClient_Subscribe(&appData.myClient, &subscribe);
            SYS_CONSOLE_PRINT("App:  MQTT.Subscribe: %s (%d)\r\n",
            MqttClient_ReturnCodeToString(rc), rc);
            if(rc != MQTT_CODE_SUCCESS)
            {
                SYS_CONSOLE_MESSAGE("App:  MQTT.Subscribe: failed\r\n");
                APP_TIMER_Set(&appData.genericUseTimer);
                while(!APP_TIMER_Expired(&appData.genericUseTimer, 5));   
                appData.state = APP_TCPIP_ERROR;
                break;
            }
            for (i = 0; i < subscribe.topic_count; i++)
            {
                topic = &subscribe.topics[i];
                SYS_CONSOLE_PRINT("App:  MQTT.Topic List: %s, Qos %u, Return Code %u\r\n",
                    topic->topic_filter, topic->qos, topic->return_code);
            }  

            /* Publish Topic */
            XMEMSET(&publish, 0, sizeof(MqttPublish));
            publish.retain = 0;
            publish.qos = 0;
            publish.duplicate = 0;
            publish.topic_name = topic_awsUpdate;
            publish.packet_id = mqttclient_get_packetid();
            char publishPayload [MAX_BUFFER_SIZE];
            sprintf(publishPayload, "{\"state\":{\"reported\":{\"led1\":\"%s\",\"led2\":\"%s\",\"led4\":\"%s\",\"led3\":\"%s\",\"hardware\":{\"type\":\"%s\",\"firmware_version\":\"%s\"}}}}",
                    appData.led1val ? "on" : "off", appData.led2val ? "on" : "off", appData.led3val ? "on" : "off", appData.led4val ? "on" : "off", APP_HARDWARE, APP_FIRMWARE_VERSION);
            publish.buffer = (byte *)publishPayload;
            appData.led1val ? BSP_LEDOn(BSP_LED_1_CHANNEL, BSP_LED_1_PORT) : BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT); 
            appData.led2val ? BSP_LEDOn(BSP_LED_2_CHANNEL, BSP_LED_2_PORT) : BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);  
            appData.led3val ? BSP_LEDOn(BSP_LED_3_CHANNEL, BSP_LED_3_PORT) : BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);  
            appData.led4val ? BSP_LEDOn(BSP_LED_4_CHANNEL, BSP_LED_4_PORT) : BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);  
            publish.total_len = strlen((char *)publish.buffer);
            rc = MqttClient_Publish(&appData.myClient, &publish);
            SYS_CONSOLE_PRINT("App:  MQTT.Publish: Topic %s, %s (%d)\r\n    Payload: %s\r\n",
                publish.topic_name, MqttClient_ReturnCodeToString(rc), rc, publish.buffer);
            if(rc != MQTT_CODE_SUCCESS)
            {
                SYS_CONSOLE_MESSAGE("App:  MQTT.Publish: failed\r\n");   
                appData.state = APP_TCPIP_ERROR;
                break;
            }
            appData.state = APP_TCPIP_MQTT_LOOP;
            break;
        }
        
        case APP_TCPIP_MQTT_LOOP:
        {
            int rc = 0;
            if(APP_TIMER_Expired(&appData.mqttKeepAlive, KEEP_ALIVE))
            {
                /* Keep Alive */
                rc = MqttClient_Ping(&appData.myClient);
                // Reset keep alive timer
                APP_TIMER_Set(&appData.mqttKeepAlive);
                if (rc != MQTT_CODE_SUCCESS)
                {  
                    SYS_CONSOLE_PRINT("App:  MQTT.Ping:  Keep Alive Error: %s (%d)\r\n",
                    MqttClient_ReturnCodeToString(rc), rc);
                    appData.state = APP_TCPIP_ERROR;
                    break;
                } 
                else
                {
                    SYS_CONSOLE_PRINT("App:  MQTT.Ping: %s (%d)\r\n",
                        MqttClient_ReturnCodeToString(rc), rc);
                    appData.lightShowVal = BSP_LED_TX;
                    xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);
                }
            }
            
            // Check for incoming messages
            rc = MqttClient_WaitMessage(&appData.myClient, 50);
            
            if (rc == MQTT_CODE_ERROR_TIMEOUT)
            {
                /* Keep Alive */
                rc = MqttClient_Ping(&appData.myClient);
                if (rc != MQTT_CODE_SUCCESS)
                {
                    SYS_CONSOLE_PRINT("App:  MQTT.Ping:  Keep Alive Error: %s (%d)\r\n",
                        MqttClient_ReturnCodeToString(rc), rc);
                    appData.state = APP_TCPIP_ERROR;
                    break;
                } 
            }
            else if (rc == MQTT_CODE_ERROR_NETWORK)
            {
                SYS_CONSOLE_PRINT("App:  MQTT.WaitMessage:  Network Error: %s (%d)\r\n",
                MqttClient_ReturnCodeToString(rc), rc);
                appData.state = APP_TCPIP_ERROR;
                break;
            }
            else if (rc == APP_CODE_ERROR_CMD_TIMEOUT)
            {
                bool switchPublish, potPublish;
                switchPublish = potPublish = false;
                uint32_t potVal;
                struct switchMessage test;
                
                /* There was no received messages check for publishes */
                if( uxQueueMessagesWaiting( app1Data.potentiometerQueue ) > 0 )
                {
                    xQueueReceive( app1Data.potentiometerQueue, &potVal, 1 );
                    potPublish = true;
                }
                if( uxQueueMessagesWaiting( app1Data.switchQueue ) > 0 )
                {
                    xQueueReceive( app1Data.switchQueue, &test, 1 );
                    switchPublish = true;
                }
                if(switchPublish || potPublish)
                {
                    //Got switch change, send a publish
                    JSON_Value *rootValue = json_value_init_object();
                    JSON_Object *rootObj = json_value_get_object(rootValue);
                    char *serialized_string = NULL;
                    char reportedPayload[MAX_BUFFER_SIZE];
                    if(switchPublish)
                    {
                        json_object_dotset_string(rootObj, APP_Switch_Publish_Helper(test.switchNum), (test.switchVal ? "up" : "down"));         
                    }
                    if(potPublish)
                    {
                        char potString[5] = {'\0'};
                        sprintf(potString, "%d", potVal);
                        json_object_dotset_string(rootObj, "state.reported.potentiometer", potString);
                    }
                    serialized_string = json_serialize_to_string(rootValue);
                    strcpy(reportedPayload, serialized_string);
                    json_free_serialized_string(serialized_string);

                    /* Publish Topic */
                    MqttPublish publish;
                    int rc;
                    XMEMSET(&publish, 0, sizeof(MqttPublish));
                    publish.retain = 0;
                    publish.qos = 0;
                    publish.duplicate = 0;
                    /* Build list of topics */
                    publish.topic_name = topic_awsUpdate;
                    publish.packet_id = mqttclient_get_packetid();
                    publish.buffer = (byte *)reportedPayload;
                    publish.total_len = strlen((char *)publish.buffer);
                    rc = MqttClient_Publish(&appData.myClient, &publish);
                    SYS_CONSOLE_PRINT("App:  MQTT.Publish: Topic %s, %s (%d)\r\n    Payload: %s\r\n",
                        publish.topic_name, MqttClient_ReturnCodeToString(rc), rc, publish.buffer);
                    if (rc != MQTT_CODE_SUCCESS)
                    {
                        SYS_CONSOLE_MESSAGE("App:  MQTT.Publish: failed, closing socket and reconnecting\r\n\r\n");
                        appData.state = APP_TCPIP_ERROR;
                    }
                    appData.lightShowVal = BSP_LED_TX;
                    xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);
                    json_value_free(rootValue);
                    // Reset keep alive timer since we sent a publish
                    APP_TIMER_Set(&appData.mqttKeepAlive);
                }

                break;
            }
            else if( rc != MQTT_CODE_SUCCESS)
            {
                appData.lightShowVal = BSP_LED_SERVER_CONNECT_FAILED;
                xQueueSendToFront(app1Data.lightShowQueue, &appData.lightShowVal, 1);  
                SYS_CONSOLE_PRINT("App: MQTT.WaitMessage:  Error: %s (%d)\r\n", MqttClient_ReturnCodeToString(rc), rc);
                appData.state = APP_TCPIP_ERROR;
            }
            break;
        }
        
        case APP_TCPIP_ERROR:
        {
            SYS_CONSOLE_PRINT("App: Closing Socket %d\r\n\r\n", appData.socket);
            NET_PRES_SocketClose(appData.socket);
            appData.state = APP_TCPIP_MQTT_NET_CONNECT;
            break;
        }
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
