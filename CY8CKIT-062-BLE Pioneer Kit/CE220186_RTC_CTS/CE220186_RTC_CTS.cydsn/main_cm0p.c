/******************************************************************************
* File Name: main_cm0p.c
*
* Version: 1.10
*
* Description: Cortex-M0+ starts BLE, starts the Cortex-M4, and periodically 
*              processes BLE stack events. Majority of the processing is done 
*              by the Cortex-M4. See main_cm4.c for details.
*
* Related Document: CE220186_RTC_CTS.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD EINK Display Shield
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
* Cortex-M0+ starts BLE, starts the Cortex-M4, and periodically processes BLE 
* stack events. Majority of the processing is done by the Cortex-M4. 
* See main_cm4.c for details.
*******************************************************************************/

/* Header file includes */
#include <project.h>

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function enables the Cortex-M4 and continuously 
*  processes BLE controller events
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* Enable global interrupts */
    __enable_irq();
    
    /* Start the Controller portion of BLE. Host runs on the Cortex-M4 */
    if(Cy_BLE_Start(NULL) == CY_BLE_SUCCESS)
    {
        /* Enable Cortex-M4 only if BLE Controller started successfully. 
           CY_CORTEX_M4_APPL_ADDR must be updated if Cortex-M4 memory layout 
           is changed. */
        Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 
    }
    else
    {
        /* Halt the CPU if the BLE initialization failed */
        CY_ASSERT(0u != 0u);
    }
    
    for (;;)
    {
        /* Cy_Ble_ProcessEvents() allows the BLE stack to process pending events */
        Cy_BLE_ProcessEvents();
        
        /* Put Cortex-M0+ to Deep-Sleep mode. The BLE Controller automatically wakes  
           up the CPU if processing is required */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}

/* [] END OF FILE */
