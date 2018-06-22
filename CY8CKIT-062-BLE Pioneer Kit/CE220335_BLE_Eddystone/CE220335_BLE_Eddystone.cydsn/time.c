/******************************************************************************
* File Name: time.c
*
* Version: 1.10
*
* Description: This file contains functions that configure a MCWDT and  
*              associated interrupts for time keeping and returns time
*              elapsed since power-on or reset
*
* Related Document: CE220335_BLE_Eddystone.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
*
*******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
/******************************************************************************
* This file contains functions that configure a MCWDT and associated interrupts 
* for time keeping and returns time elapsed since power-on or reset
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "time.h"

/* Data type that stores the time elapsed since power-up or reset */
typedef union
{
    /* 32-bit second count for mathematical operations */
    uint32_t    count;
    /* Same second count segmented into four 8-bit fields for copying 
       to the Eddystone frame */
    uint8_t     countByte[4u];   
}   second_count_t;

/* Variable  that stores the time elapsed in seconds since power-up or reset */
second_count_t seconds;

/*******************************************************************************
* Function Name: void IsrMcwdt(void)
********************************************************************************
*
* Summary:
*  Interrupt service routine for the MCWDT interrupt
*    
* Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void IsrMcwdt(void)
{
    /* Clear the MCWDT interrupt */
    Cy_MCWDT_ClearInterrupt(MCWDT_HW,CY_MCWDT_CTR0);
    NVIC_ClearPendingIRQ(isr_MCWDT_cfg.intrSrc);  
    
    /* Increment the seconds count (100 ms per bit) */
    seconds.count++;
}

/*******************************************************************************
* Function Name: void startSecondCount(void)
********************************************************************************
*
* Summary:
*  Starts the components used for time keeping
*    
* Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void startSecondCount(void)
{
    /* Initialize MCDWT Counter0. Period of Counter0 is set to 100 ms inside the
       component on the schematic */
    Cy_MCWDT_Init(MCWDT_HW, &MCWDT_config);
    /* Enable the interrupts for MCDWT Counter0 only */
    Cy_MCWDT_SetInterruptMask(MCWDT_HW, CY_MCWDT_CTR0);
    
    /* Initialize and enable MCWDT interrupt */
    Cy_SysInt_Init(&isr_MCWDT_cfg,IsrMcwdt);
    NVIC_ClearPendingIRQ(isr_MCWDT_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_MCWDT_cfg.intrSrc);
    
    /* Clear the seconds count */
    seconds.count = 0x00000000u;
    
    /* Start MCDWT Counter0 */
    Cy_MCWDT_Enable(MCWDT_HW, MCWDT_ENABLED_CTRS_MASK,0u); 
}

/*******************************************************************************
* Function Name: uint8_t* getSecondCount (void)
********************************************************************************
*
* Summary:
*  This function returns time elapsed since power-on or reset
*    
* Parameters:
*  None
*
* Return:
*  uint8_t* : Pointer to a 4-byte array storing time elapsed since power-on or
*             reset 
*
**********************************************************************************/
uint8_t* getSecondCount (void)
{
    /* Return seconds count (100 ms per bit) */
    return seconds.countByte;
}

/* [] END OF FILE */
