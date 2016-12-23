/*******************************************************************************
  CMP Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_cmp_static.c

  Summary:
    CMP driver implementation for the static single instance driver.

  Description:
    The CMP device driver provides a simple interface to manage the CMP
    modules on Microchip microcontrollers.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
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

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "framework/driver/cmp/drv_cmp_static.h"


// *****************************************************************************
// *****************************************************************************
// Section: CMP static driver functions
// *****************************************************************************
// *****************************************************************************
void DRV_CMP_Initialize(void)
{

    /* Configure CVREF for comparator use. */
    PLIB_CMP_CVREF_SourceVoltageSelect(CMP_ID_1, CMP_CVREF_VOLTAGE_SOURCE_VDD);
    #if defined(PLIB_CMP_ExistsCVREFRefVoltageRangeSelect )
        if ( PLIB_CMP_ExistsCVREFRefVoltageRangeSelect  ( CMP_ID_1 ) )
        {
		    PLIB_CMP_CVREF_ReferenceVoltageSelect ( CMP_ID_1,  CMP_CVREF_RESISTOR_LADDER_VOLTAGE );
        }
        else
        {
            /* If Voltage reference selection for CVref feature doesn't exist 
			on CVREF module instance,
            then by default Resister Latter Network is selected as reference, so do nothing */
        }
    #endif	
    PLIB_CMP_CVREF_ValueSelect(CMP_ID_1, CMP_CVREF_VALUE_11);
    #if defined(PLIB_CMP_ExistsCVREFBGRefVoltageRangeSelect )
        if ( PLIB_CMP_ExistsCVREFBGRefVoltageRangeSelect  ( CMP_ID_1 ) )
        {
		    PLIB_CMP_CVREF_BandGapReferenceSourceSelect ( CMP_ID_1,  CMP_CVREF_BANDGAP_1_2V );
        }
        else
        {
            /* If Voltage reference selection for IVref feature doesn't exist 
			on CVREF module instance,
            then by default internal 1.2V is selected as reference, so do nothing */
        }
    #endif	
    PLIB_CMP_CVREF_Enable(CMP_ID_1);

    PLIB_CMP_NonInvertingInputChannelSelect(CMP_ID_2, CMP_NON_INVERTING_INPUT_CVREF);
    PLIB_CMP_InvertingInputChannelSelect(CMP_ID_2, CMP_INVERTING_INPUT_EXTERNAL_PIN_B);

    PLIB_CMP_OutputInvertDisable(CMP_ID_2);
    PLIB_CMP_OutputEnable(CMP_ID_2);

    /* Setup Interrupt */
    PLIB_CMP_InterruptEventSelect(CMP_ID_2, CMP_INTERRUPT_GENERATION_LOW_TO_HIGH);   
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_COMPARATOR_2);
    PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_CMP2, INT_PRIORITY_LEVEL1);
    PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_CMP2, INT_SUBPRIORITY_LEVEL0);          

    PLIB_CMP_Enable(CMP_ID_2);

}

/*******************************************************************************
 End of File
*/
