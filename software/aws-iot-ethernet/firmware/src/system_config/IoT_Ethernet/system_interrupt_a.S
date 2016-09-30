/*******************************************************************************
 System Interrupt Source File

  File Name:
    sys_interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the 
    interrupt sub-system.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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
/*
*********************************************************************************************************
*                                           INCLUDES
*********************************************************************************************************
*/
#include <xc.h>



#include "ISR_Support.h"



/* TMR Instance 0 Interrupt */
   .extern  IntHandlerDrvTmrInstance0

   .section	.vector_9,code, keep
   .equ     __vector_dispatch_9, IntVectorDrvTmrInstance0
   .global  __vector_dispatch_9
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvTmrInstance0

IntVectorDrvTmrInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvTmrInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvTmrInstance0



/* NVM Interrupt */
   .extern  IntHandlerDrvNvm

   .section	.vector_167,code, keep
   .equ     __vector_dispatch_167, IntVectorDrvNvm
   .global  __vector_dispatch_167
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvNvm

IntVectorDrvNvm:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvNvm
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvNvm


/* CMP Instance 0 Interrupt */
   .extern  IntHandlerDrvCMPInstance0

   .section	.vector_131,code, keep
   .equ     __vector_dispatch_131, IntVectorDrvCMPInstance0
   .global  __vector_dispatch_131
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvCMPInstance0

IntVectorDrvCMPInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvCMPInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvCMPInstance0



/* USB Device Interrupt */
   .extern  IntHandlerUSBInstance0

   .section	.vector_132,code, keep
   .equ     __vector_dispatch_132, IntVectorUSBInstance0
   .global  __vector_dispatch_132
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorUSBInstance0

IntVectorUSBInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerUSBInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorUSBInstance0


   .extern  IntHandlerUSBInstance0_USBDMA

   .section	.vector_133,code, keep
   .equ     __vector_dispatch_133, IntVectorUSBInstance0_USBDMA
   .global  __vector_dispatch_133
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorUSBInstance0_USBDMA

IntVectorUSBInstance0_USBDMA:
    portSAVE_CONTEXT
    la    s6,  IntHandlerUSBInstance0_USBDMA
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorUSBInstance0_USBDMA



/* ETHMAC Interrupt */
   .extern  IntHandler_ETHMAC

   .section	.vector_153,code, keep
   .equ     __vector_dispatch_153, IntVector_ETHMAC
   .global  __vector_dispatch_153
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVector_ETHMAC

IntVector_ETHMAC:
    portSAVE_CONTEXT
    la    s6,  IntHandler_ETHMAC
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVector_ETHMAC



/*******************************************************************************
 End of File
 */

