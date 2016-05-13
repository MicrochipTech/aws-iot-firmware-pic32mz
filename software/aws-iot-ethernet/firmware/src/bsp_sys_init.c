/*******************************************************************************
  Board Support Package Implementation.

  Company:      
    Microchip Technology Inc.

  File Name:    
    bsp_sys_init.c

  Summary:      
    Board Support Package Implementation for IoT Wi-Fi n AWS IoT board.

  Description:
    This file contains the implementation of the Board Support Package for the
    IoT Wi-Fi n AWS IoT board to help interface with the board.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#include "bsp_config.h"
#include "app.h"

#define BSP_SWITCH_MS_ELLAPSED_TIME_TO_HZ(x) (1250/(x)) // convert time to frequency

BSP_DATA bspData;

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function: void BSP_Initialize(void)

  Summary:
    Performs the neccassary actions to initialize a board

  Description:
    This routine performs the neccassary actions to initialize a board

  Remarks:
    Refer to bsp_config.h for usage information.

*/

void BSP_Initialize(void )
{
    /* Initialize the LED light show value and switch states*/
    bspData.light_show      = BSP_LED_EASY_CONFIGURATION;
    bspData.previousStateS1 = BSP_SWITCH_STATE_DEASSERTED;
    bspData.previousStateS2 = BSP_SWITCH_STATE_DEASSERTED;
    bspData.previousStateS3 = BSP_SWITCH_STATE_DEASSERTED;
    bspData.previousStateS4 = BSP_SWITCH_STATE_DEASSERTED;
    bspData.s1              = BSP_SWITCH_STATE_DEASSERTED;
    bspData.s2              = BSP_SWITCH_STATE_DEASSERTED;
    bspData.s3              = BSP_SWITCH_STATE_DEASSERTED;
    bspData.s4              = BSP_SWITCH_STATE_DEASSERTED;

    /* Initialize switch state machine values for each switch object */
    int i;
    for(i=0 ; i<BSP_MAX_SWITCHES; i++)
    {
       bspData.switches[i].duration     =  BSP_SWITCH_DEBOUNCE_TIME;
       bspData.switches[i].startTick    = 0;
       bspData.switches[i].endTick      = 0;
       bspData.switches[i].timerActive  = false;
    }
}

// *****************************************************************************
/* Function: 
    void BSP_LEDStateSet(BSP_LED_PORT led_port, BSP_LED_CHANNEL led_channel, 
        BSP_LED_STATE led_state)


  Summary:
    Controls the state of the LED.
  
  Description:
    This function allows the application to specify the state of the LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LEDStateSet(BSP_LED_PORT led_port, BSP_LED_CHANNEL led_channel, BSP_LED_STATE led_state)
{
    PLIB_PORTS_PinWrite( PORTS_ID_0, led_channel, led_port, led_state );
}

// *****************************************************************************
/* Function: 
    void BSP_LEDOn(BSP_LED_CHANNEL led_channel, BSP_LED_PORT led_port);

  Summary:
    Switches ON the specified LED.
  
  Description:
    This function switches ON the specified LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LEDOn(BSP_LED_CHANNEL led_channel, BSP_LED_PORT led_port)
{
    PLIB_PORTS_PinSet( PORTS_ID_0, led_channel, led_port );
}

// *****************************************************************************
/* Function: 
    void BSP_LEDOff(BSP_LED_CHANNEL led_channel, BSP_LED_PORT led_port);

  Summary:
    Switches OFF the specified LED.
  
  Description:
    This function switches OFF the specified LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LEDOff(BSP_LED_CHANNEL led_channel, BSP_LED_PORT led_port)
{
    PLIB_PORTS_PinClear( PORTS_ID_0, led_channel, led_port );
}

// *****************************************************************************
/* Function:
    void BSP_LEDAllOff();

  Summary:
    Switches OFF all LEDs.

  Description:
    This function switches OFF all LEDs.

  Remarks:
    Refer to bsp_config.h for usage information.
*/
void BSP_LEDAllOff()
{
    BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
    BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
    BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
    BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
    BSP_LEDOff(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
    BSP_LEDOff(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
    BSP_LEDOff(BSP_LED_7_CHANNEL, BSP_LED_7_PORT);
}

// *****************************************************************************
/* Function: 
    BSP_LED_STATE BSP_LEDStateGet(BSP_LED_CHANNEL led_channel, BSP_LED_PORT led_port);

  Summary:
    Returns the present state of the LED.
  
  Description:
    This function returns the present state of the LED.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

BSP_LED_STATE BSP_LEDStateGet(BSP_LED_CHANNEL led_channel, BSP_LED_PORT led_port)
{
    return(PLIB_PORTS_PinGetLatched( PORTS_ID_0, led_channel, led_port));
}

// *****************************************************************************
/* Function: 
    void BSP_LEDToggle(BSP_LED_CHANNEL led_channel, BSP_LED_PORT led_port);

  Summary:
    Toggles the state of the LED between BSP_LED_STATE_ON and BSP_LED_STATE_OFF.
  
  Description:
    This function toggles the state of the LED between BSP_LED_STATE_ON and
    BSP_LED_STATE_OFF.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LEDToggle(BSP_LED_CHANNEL led_channel, BSP_LED_PORT led_port)
{
    PLIB_PORTS_PinToggle( PORTS_ID_0, led_channel, led_port);
}

// *****************************************************************************
/* Function: 
    void BSP_SWITCH_StateGet(BSP_SWITCH_CHANNEL bspSwitchChannel, BSP_SWITCH_PORT bspSwitchPort);

  Summary:
    Returns the present state (pressed or not pressed) of the specified switch.
  
  Description:
    This function returns the present state (pressed or not pressed) of the
    specified switch.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

BSP_SWITCH_STATE BSP_SWITCH_StateGet( BSP_SWITCH_CHANNEL bspSwitchChannel, BSP_SWITCH_PORT bspSwitchPort )
{
    return (PLIB_PORTS_PinGet(PORTS_ID_0, bspSwitchChannel, bspSwitchPort));
}


// *****************************************************************************
/* Function:
    void BSP_LED_SetLightShow(BSP_LED_LIGHT_SHOW lightShow);

  Summary:
    Sets the LED light show to a configuration

  Description:
    This function will change the BSP LED configuration to certain "light shows"
    These light shows will indicate certain functions on the board.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_LED_LightShowSet(BSP_LED_LIGHT_SHOW lightShow)
{
    bspData.light_show = lightShow;
}

// *****************************************************************************
/* Function:
    void BSP_SYS_Tasks();

  Summary:
    Runs the applications BSP tasks such as LED and Switch control

  Description:
    This function will handle the LED state machine and switch state machine.

  Remarks:
    Refer to bsp_config.h for usage information.
*/

void BSP_SYS_Tasks ()
{
    BSP_LED_LightShow(bspData.light_show);
    BSP_SWITCH_Tasks();
}

// *****************************************************************************
/* Function:
    void BSP_LED_LightShow(void)

  Summary:
     Polling function to call different LED light shows

  Description:
    Each function will toggle the LEDs with different "light shows" that can be used
    for indications such as an error

  Precondition:
    Tick system is setup and LEDs are initialized

  Parameters:
   None

  Returns:
   None
*/

void BSP_LED_LightShow(BSP_LED_LIGHT_SHOW lightShow)
{
    static uint32_t ledTick = 0;
    static uint8_t count = 0;
    static uint8_t count2 = 0;
    static uint32_t connectionTimeout = 0;

    switch (lightShow)
    {
        case     BSP_LED_EASY_CONFIGURATION:
            if(SYS_TMR_TickCountGet() - ledTick >= 625)
            {
                ledTick = SYS_TMR_TickCountGet();
                BSP_LEDToggle(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
                BSP_LEDOff(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
                BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
            }
            break;

        case     BSP_LED_CONNECTING_TO_AP:
            if(SYS_TMR_TickCountGet() - connectionTimeout >= (1250 * 10))
            {
                connectionTimeout = SYS_TMR_TickCountGet();
                BSP_LED_LightShowSet(BSP_LED_CONNECTION_FAILED);
            }
            if (SYS_TMR_TickCountGet() - ledTick >= 150)
            {
                ledTick = SYS_TMR_TickCountGet();
                BSP_LEDOff(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
                BSP_LEDOn(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
                switch (count)
                {
                    case (0):
                        BSP_LEDOn(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                        BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                        BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                        BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                        count++;
                        break;
                    case (1):
                        BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                        BSP_LEDOn(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                        BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                        BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                        count++;
                        break;
                    case (2):
                        BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                        BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                        BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                        BSP_LEDOn(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                        count++;
                        break;
                    case (3):
                        BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                        BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                        BSP_LEDOn(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                        BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                        count = 0;
                        break;
                    default:
                        count = 0;
                        break;
                }

            }
            break;
            //Not implemented
        case     BSP_LED_CONNECTION_FAILED:
                BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                BSP_LEDOn(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                BSP_LEDOn(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
                BSP_LEDOn(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);

            if(SYS_TMR_TickCountGet() - connectionTimeout >= 4750)
            {
                connectionTimeout = SYS_TMR_TickCountGet();
                ledTick = SYS_TMR_TickCountGet();
                BSP_LED_LightShowSet(BSP_LED_CONNECTING_TO_AP);
            }
            break;
        case    BSP_LED_AP_CONNECTED:
                if(SYS_TMR_TickCountGet() - ledTick >= 625)
                {
                    count2++;
                    ledTick = SYS_TMR_TickCountGet();
                    BSP_LEDStateSet(BSP_LED_5_CHANNEL, BSP_LED_5_PORT, BSP_LEDStateGet(BSP_LED_6_CHANNEL, BSP_LED_6_PORT));
                    BSP_LEDToggle(BSP_LED_5_CHANNEL, BSP_LED_6_PORT);
                    BSP_LEDToggle(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
                    BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                    BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                    BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                    BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                }
            break;

        case    BSP_LED_SERVER_CONNECT_FAILED:
                BSP_LEDOn(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
                BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
                BSP_LEDOff(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
                BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
                BSP_LEDOn(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
                BSP_LEDOn(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
            break;

        case    BSP_LED_ALL_GOOD:
                BSP_LEDOff(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
                BSP_LEDOff(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
                ledTick = SYS_TMR_TickCountGet();
            // Idle state
            break;
            
        case    BSP_LED_TX:
            BSP_LEDOn(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
            if(SYS_TMR_TickCountGet() - ledTick >= 30)
                BSP_LED_LightShowSet(BSP_LED_ALL_GOOD);
            break;
            
        case    BSP_LED_RX:
            BSP_LEDOn(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
            if(SYS_TMR_TickCountGet() - ledTick >= 30)
                BSP_LED_LightShowSet(BSP_LED_ALL_GOOD);
            break; 

        case    BSP_LED_DNS_FAILED:
            BSP_LEDOn(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
            BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
            BSP_LEDOn(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
            BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
            BSP_LEDOn(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
            BSP_LEDOn(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
        break;
        
        case    BSP_LED_TCPIP_STACK_INIT_FAILURE:
            BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
            BSP_LEDOn(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
            BSP_LEDOn(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
            BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
            BSP_LEDOn(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
            BSP_LEDOn(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
        break;
        
        case BSP_LED_NVM_FAILED_MOUNT:
            BSP_LEDOff(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
            BSP_LEDOff(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
            BSP_LEDOn(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
            BSP_LEDOff(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
            BSP_LEDOn(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
            BSP_LEDOn(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
        break;  
        
        case BSP_LED_INTIAL_CONNECT:
            BSP_LEDOn(BSP_LED_1_CHANNEL, BSP_LED_1_PORT);
            BSP_LEDOn(BSP_LED_2_CHANNEL, BSP_LED_2_PORT);
            BSP_LEDOn(BSP_LED_3_CHANNEL, BSP_LED_3_PORT);
            BSP_LEDOn(BSP_LED_4_CHANNEL, BSP_LED_4_PORT);
            BSP_LEDOn(BSP_LED_5_CHANNEL, BSP_LED_5_PORT);
            BSP_LEDOn(BSP_LED_6_CHANNEL, BSP_LED_6_PORT);
            
        default:
            break;
    }
}

BSP_SWITCH_STATE BSP_SWITCH_SwitchGetState(BSP_SWITCH_PORT switchId)
{
    switch(switchId)
    {
        case BSP_SWITCH_1_PORT:
          return (BSP_SWITCH_STATE)bspData.s1;
        break;
        case BSP_SWITCH_2_PORT:
          return (BSP_SWITCH_STATE)bspData.s2;
        break;
        case BSP_SWITCH_3_PORT:
          return (BSP_SWITCH_STATE)bspData.s3;
        break;
        case BSP_SWITCH_4_PORT:
          return (BSP_SWITCH_STATE)bspData.s4;
        break;
        default:
            return BSP_SWITCH_STATE_DEASSERTED;
    }
}

void BSP_SWITCH_SwitchSetPreviousState(BSP_SWITCH_PORT switchId, BSP_SWITCH_STATE var)
{
    switch(switchId)
    {
        case BSP_SWITCH_1_PORT:
          bspData.previousStateS1 = var;
        break;
        case BSP_SWITCH_2_PORT:
          bspData.previousStateS2 = var;
        break;
        case BSP_SWITCH_3_PORT:
          bspData.previousStateS3 = var;
        break;
        case BSP_SWITCH_4_PORT:
          bspData.previousStateS4 = var;
        break;
        default:
             ;
    }
}

int32_t BSP_SWITCH_DeviceDebounce(BSP_SWITCH_DEBOUNCE_T *handle, uint32_t curVal)
{
    if(handle->timerActive==false)
    {
       handle->timerActive = true;
       handle->prevValue   = curVal;
       handle->startTick   = SYS_TMR_TickCountGet();
    }

    if ((SYS_TMR_TickCountGet() - handle->startTick) >= (1250
                         / BSP_SWITCH_MS_ELLAPSED_TIME_TO_HZ(handle->duration)) )
    {
        handle->timerActive=false;
        if(handle->prevValue == curVal)
          return handle->prevValue;
        else
          return curVal; // Return the assert value
    }
    return BSP_SWITCH_BUSY;  // Busy
}

void BSP_SWITCH_Tasks(void)
{
    int32_t val;
    val = BSP_SWITCH_DeviceDebounce(&bspData.switches[0], 
        BSP_SWITCH_StateGet( BSP_SWITCH_1_CHANNEL, BSP_SWITCH_1_PORT));
    if(val != BSP_SWITCH_BUSY)
    {
        bspData.s1 = val;
    }

    val = BSP_SWITCH_DeviceDebounce(&bspData.switches[1],
        BSP_SWITCH_StateGet( BSP_SWITCH_2_CHANNEL, BSP_SWITCH_2_PORT));
    if(val != BSP_SWITCH_BUSY) 
    {
        bspData.s2 = val;
    }

    val = BSP_SWITCH_DeviceDebounce(&bspData.switches[2],
        BSP_SWITCH_StateGet( BSP_SWITCH_3_CHANNEL, BSP_SWITCH_3_PORT));
    if(val != BSP_SWITCH_BUSY) 
    {
        bspData.s3 = val;
    }

    val = BSP_SWITCH_DeviceDebounce(&bspData.switches[3],
        BSP_SWITCH_StateGet( BSP_SWITCH_4_CHANNEL, BSP_SWITCH_4_PORT));
    if(val != BSP_SWITCH_BUSY) 
    {
        bspData.s4 = val;
    }
}

/*******************************************************************************
 End of File
*/
