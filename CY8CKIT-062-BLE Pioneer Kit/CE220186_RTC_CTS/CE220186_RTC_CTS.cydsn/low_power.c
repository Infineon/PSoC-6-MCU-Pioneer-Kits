/******************************************************************************
* File Name: low_power.c
*
* Version: 1.10
*
* Description: This file contains functions that make the system enter low
*              power modes and turn off the status LEDs depending on the
*              system level conditions
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
* This file contains functions that make the system enter low power modes and 
* turn off the status LEDs depending on the system level conditions
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include "low_power.h"
#include "ble_application.h"
#include "led.h"

/*******************************************************************************
* Function Name: void IsrMcwdt(void)
********************************************************************************
*
* Summary:
*  Interrupt service routine for the MCWDT interrupt
*    
*  Parameters:
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

    /* Update the status LEDs and turn them off if required */    
    ServiceStatusLEDs();
}

/*******************************************************************************
* Function Name: void isrGPIO(void)
********************************************************************************
*
* Summary:
*  Interrupt service routine for the port interrupt triggered from isr_gpio.
*    
*  Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void isrGPIO(void)
{
    /* Clear the GPIO interrupt*/
    Cy_GPIO_ClearInterrupt(Advertise_0_PORT,Advertise_0_NUM);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);  
    
    /* Restart BLE advertisement */
    RestartBleAvertisement();
}

/*******************************************************************************
* Function Name: void InitLowPower(void)
********************************************************************************
*
* Summary:
*  Initializes the components used for low power functions
*    
*  Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void InitLowPower(void)
{
    /* Start MCDWT Counter0 */
    Cy_MCWDT_Init(MCWDT_HW, &MCWDT_config);
    /* Enable the interrupts for MCDWT Counter0 only */
    Cy_MCWDT_SetInterruptMask(MCWDT_HW, CY_MCWDT_CTR0);
    
    /* Initialize and enable MCWDT interrupt */
    Cy_SysInt_Init(&isr_MCWDT_cfg,IsrMcwdt);
    NVIC_ClearPendingIRQ(isr_MCWDT_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_MCWDT_cfg.intrSrc);
    
    /* Start MCDWT Counter0 */
    Cy_MCWDT_Enable(MCWDT_HW, MCWDT_ENABLED_CTRS_MASK,0u);
    
    /* Initialize and enable the GPIO interrupt assigned to CM4 */
    Cy_SysInt_Init(&isr_gpio_cfg,isrGPIO);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_gpio_cfg.intrSrc);
}

/*******************************************************************************
* Function Name: void HandleLowPowerMode(void)
********************************************************************************
*
* Summary:
*  Evaluates the status of the system and enters low power mode if the conditions
*  permit
*
*  Parameters:
*  None
*
* Return:
*  None
*
**********************************************************************************/
void HandleLowPowerMode(void)
{
    /* Make sure that BLE is ready to enter low power mode */
    if(IsBleReadyForLowPowerMode() == true)
    {
        /* Disable MCDWT Counter0 to prevent periodic wake-ups if none of the
           status LEDs need periodic toggle */
        if(IsLEDperiodicToggleActive() == false)
        {
            Cy_MCWDT_Disable(MCWDT_HW, MCWDT_ENABLED_CTRS_MASK,0u);
            NVIC_DisableIRQ((IRQn_Type)isr_MCWDT_cfg.intrSrc);
        }

        /* Enter Deep-sleep mode */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

        /* Re-enable MCDWT Counter0 */
        if(IsLEDperiodicToggleActive() == false)
        {
            NVIC_EnableIRQ((IRQn_Type)isr_MCWDT_cfg.intrSrc);
            Cy_MCWDT_Enable(MCWDT_HW, MCWDT_ENABLED_CTRS_MASK,0u);
        }
    }
}
/* [] END OF FILE */
