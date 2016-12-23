/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_nvm_support.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    Contains support functions for writing application data to NVM.
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

#include "app_nvm_support.h"


extern APP_DATA appData;

bool APP_NVM_Erase(uint32_t nvm_dest_address)
{
    int tmp;
    tmp = nvm_dest_address * (appData.gAppNVMMediaGeometry->geometryTable[2].numBlocks) / (DRV_NVM_MEDIA_SIZE * 1024);
    
    DRV_NVM_Erase(appData.nvmHandle, &appData.nvmCommandHandle[0],  tmp, 1);
    if(appData.nvmCommandHandle == DRV_NVM_COMMAND_HANDLE_INVALID)
    {
        return false;
    }

    while(DRV_NVM_COMMAND_COMPLETED != DRV_NVM_CommandStatus(appData.nvmHandle, appData.nvmCommandHandle[0]))
    {
        ;
    }
    
    return true;
}

bool APP_NVM_Write(uint32_t nvm_dest_address, uint8_t * data)
{   
    int tmp;            
    tmp = nvm_dest_address * (appData.gAppNVMMediaGeometry->geometryTable[1].numBlocks) / (DRV_NVM_MEDIA_SIZE * 1024);
    DRV_NVM_EraseWrite(appData.nvmHandle, &appData.nvmCommandHandle[0], (uint8_t *)data, tmp, 1);
    if(DRV_NVM_COMMAND_HANDLE_INVALID == appData.nvmCommandHandle[0])
    {
        return false;
    }

    return true;
}

bool APP_NVM_Read(uint32_t nvm_dest_address, uint8_t * buffer, uint32_t bufferLength)
{
    DRV_NVM_Read(appData.nvmHandle, &appData.nvmCommandHandle[0], buffer, nvm_dest_address, bufferLength);
    if(DRV_NVM_COMMAND_HANDLE_INVALID == appData.nvmCommandHandle[0])
    {
        return false;
    }    

    while(DRV_NVM_COMMAND_COMPLETED != DRV_NVM_CommandStatus(appData.nvmHandle, appData.nvmCommandHandle[0]))
    {
        ;
    }

    return true;
}
