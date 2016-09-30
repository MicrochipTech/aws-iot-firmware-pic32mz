/*******************************************************************************
  ADC Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_adc_static.h

  Summary:
    ADC driver interface declarations for the static single instance driver.

  Description:
    The ADC device driver provides a simple interface to manage the ADC
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the ADC driver.
    
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

#ifndef _DRV_ADC_STATIC_H
#define _DRV_ADC_STATIC_H

#include <stdbool.h>
#include "system_config.h"
#include "peripheral/adchs/plib_adchs.h"

typedef enum {

    DRV_ADC_ID_1 = ADCHS_ID_0,
    DRV_ADCHS_NUMBER_OF_MODULES

} DRV_ADC_MODULE_ID;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for ADC Static Driver
// *****************************************************************************
// *****************************************************************************
void DRV_ADC_Initialize(void);

inline void DRV_ADC_DeInitialize(void);

inline void DRV_ADC0_Open(void);

inline void DRV_ADC0_Close(void);

inline void DRV_ADC_Start(void);

inline void DRV_ADC_Stop(void);


uint32_t DRV_ADC_SamplesRead(uint8_t bufIndex);

bool DRV_ADC_SamplesAvailable(uint8_t bufIndex);





inline void DRV_ADC_DigitalFilter0_Open(void);

inline void DRV_ADC_DigitalFilter0_Close(void);

inline bool DRV_ADC_DigitalFilter0_DataIsReady(void);

inline int16_t DRV_ADC_DigitalFilter0_DataRead(void);




#endif // #ifndef _DRV_ADC_STATIC_H

/*******************************************************************************
 End of File
*/
