/*******************************************************************************
  SYS PORTS Static Functions for PORTS System Service

  Company:
    Microchip Technology Inc.

  File Name:
    sys_ports_static.c

  Summary:
    SYS PORTS static function implementations for the Ports System Service.

  Description:
    The Ports System Service provides a simple interface to manage the ports
    on Microchip microcontrollers. This file defines the static implementation for the 
    Ports System Service.
    
  Remarks:
    Static functions incorporate all system ports configuration settings as
    determined by the user via the Microchip Harmony Configurator GUI.  It provides 
    static version of the routines, eliminating the need for an object ID or 
    object handle.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#include "system_config.h"
#include "peripheral/ports/plib_ports.h"
#include "peripheral/int/plib_int.h"


void SYS_PORTS_Initialize(void)
{
    
    /* PORT B Initialization */
    PLIB_PORTS_OpenDrainEnable(PORTS_ID_0, PORT_CHANNEL_B, SYS_PORT_B_ODC);
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_B,  SYS_PORT_B_LAT);
    PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_B,  SYS_PORT_B_TRIS ^ 0xFFFF);
    PLIB_PORTS_ChangeNoticePerPortTurnOn(PORTS_ID_0, PORT_CHANNEL_B);
    PLIB_PORTS_ChannelModeSelect(PORTS_ID_0, PORT_CHANNEL_B, SYS_PORT_B_ANSEL ^ 0xFFFF, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_ChannelChangeNoticeEnable(PORTS_ID_0, PORT_CHANNEL_B, SYS_PORT_B_CNEN);
    PLIB_PORTS_ChannelChangeNoticePullUpEnable(PORTS_ID_0, PORT_CHANNEL_B, SYS_PORT_B_CNPU);
    PLIB_PORTS_ChannelChangeNoticePullDownEnable(PORTS_ID_0, PORT_CHANNEL_B, SYS_PORT_B_CNPD);
    
    /* PORT C Initialization */
    PLIB_PORTS_OpenDrainEnable(PORTS_ID_0, PORT_CHANNEL_C, SYS_PORT_C_ODC);
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_C,  SYS_PORT_C_LAT);
    PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_C,  SYS_PORT_C_TRIS ^ 0xFFFF);
    PLIB_PORTS_ChangeNoticePerPortTurnOn(PORTS_ID_0, PORT_CHANNEL_C);
    PLIB_PORTS_ChannelModeSelect(PORTS_ID_0, PORT_CHANNEL_C, SYS_PORT_C_ANSEL ^ 0xFFFF, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_ChannelChangeNoticeEnable(PORTS_ID_0, PORT_CHANNEL_C, SYS_PORT_C_CNEN);
    PLIB_PORTS_ChannelChangeNoticePullUpEnable(PORTS_ID_0, PORT_CHANNEL_C, SYS_PORT_C_CNPU);
    PLIB_PORTS_ChannelChangeNoticePullDownEnable(PORTS_ID_0, PORT_CHANNEL_C, SYS_PORT_C_CNPD);
    
    /* PORT E Initialization */
    PLIB_PORTS_OpenDrainEnable(PORTS_ID_0, PORT_CHANNEL_E, SYS_PORT_E_ODC);
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_E,  SYS_PORT_E_LAT);
    PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_E,  SYS_PORT_E_TRIS ^ 0xFFFF);
    PLIB_PORTS_ChangeNoticePerPortTurnOn(PORTS_ID_0, PORT_CHANNEL_E);
    PLIB_PORTS_ChannelModeSelect(PORTS_ID_0, PORT_CHANNEL_E, SYS_PORT_E_ANSEL ^ 0xFFFF, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_ChannelChangeNoticeEnable(PORTS_ID_0, PORT_CHANNEL_E, SYS_PORT_E_CNEN);
    PLIB_PORTS_ChannelChangeNoticePullUpEnable(PORTS_ID_0, PORT_CHANNEL_E, SYS_PORT_E_CNPU);
    PLIB_PORTS_ChannelChangeNoticePullDownEnable(PORTS_ID_0, PORT_CHANNEL_E, SYS_PORT_E_CNPD);
    
    /* PORT F Initialization */
    PLIB_PORTS_OpenDrainEnable(PORTS_ID_0, PORT_CHANNEL_F, SYS_PORT_F_ODC);
    PLIB_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_F,  SYS_PORT_F_LAT);
    PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_F,  SYS_PORT_F_TRIS ^ 0xFFFF);
    PLIB_PORTS_ChangeNoticePerPortTurnOn(PORTS_ID_0, PORT_CHANNEL_F);
    PLIB_PORTS_ChannelModeSelect(PORTS_ID_0, PORT_CHANNEL_F, SYS_PORT_F_ANSEL ^ 0xFFFF, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_ChannelChangeNoticeEnable(PORTS_ID_0, PORT_CHANNEL_F, SYS_PORT_F_CNEN);
    PLIB_PORTS_ChannelChangeNoticePullUpEnable(PORTS_ID_0, PORT_CHANNEL_F, SYS_PORT_F_CNPU);
    PLIB_PORTS_ChannelChangeNoticePullDownEnable(PORTS_ID_0, PORT_CHANNEL_F, SYS_PORT_F_CNPD);


    /* PPS Input Remapping */

    /* PPS Output Remapping */
    PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_C2OUT, OUTPUT_PIN_RPB9 );

    
}


/*******************************************************************************
 End of File
*/
