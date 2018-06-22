/******************************************************************************
* File Name: low_power.c
*
* Version: 1.00
*
* Description: This file contains functions that make the system enter low
*              power modes and turn off the status LEDs depending on the
*              system level conditions
*
* Related Document: CE220567_BLE_Thermometer.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*                      CY8CKIT-028-EPD E-INK Display Shield
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
/*******************************************************************************
* This file contains functions that make the system enter low power modes and 
* turn off the status LEDs depending on the system level conditions
*******************************************************************************/

/* Header file includes */
#include <project.h>
#include <stdio.h>
#include "low_power.h"
#include "ble_application.h"
#include "display.h"
#include "led.h"

/* Macro used wait 3 LFCLK cycles for RTC operations */
#define WAIT_3LFCLKS    (92u)

/* Default time written to RTC at power-up or reset */
#define TIME_AT_RESET           (00u),   /* Seconds    */\
                                (00u),   /* Minutes    */\
                                (00u),   /* Hours      */\
                                (01u),   /* Date       */\
                                (01u),   /* Month      */\
                                (17u)    /* Year 20xx  */

/* Structure that enables alarm interrupts at 1 minute intervals, i.e when the 
   seconds field rolls over to "00". Note that only the seconds field is enabled. 
   Other fields are kept at the default values and not enabled */                                
cy_stc_rtc_alarm_t const alarm = 
{
    .sec            =   00u,
    .secEn          =   CY_RTC_ALARM_ENABLE,
    .min            =   00u,
    .minEn          =   CY_RTC_ALARM_DISABLE,
    .hour           =   00u,
    .hourEn         =   CY_RTC_ALARM_DISABLE,
    .dayOfWeek      =   01u,
    .dayOfWeekEn    =   CY_RTC_ALARM_DISABLE,
    .date           =   01u,
    .dateEn         =   CY_RTC_ALARM_DISABLE,
    .month          =   01u,
    .monthEn        =   CY_RTC_ALARM_DISABLE,
    .almEn          =   CY_RTC_ALARM_ENABLE
};

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
*******************************************************************************/
void IsrMcwdt(void)
{
    /* Clear the MCWDT interrupt */
    Cy_MCWDT_ClearInterrupt(MCWDT_HW,CY_MCWDT_CTR0);
    NVIC_ClearPendingIRQ(isr_MCWDT_cfg.intrSrc);
    
    /* Request main BLE loop to read current temperature and send BLE indication
       if enabled*/
    RequestTemperatureScan();

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
*******************************************************************************/
void isrGPIO(void)
{
    /* Clear the GPIO interrupt*/
    Cy_GPIO_ClearInterrupt(Advertise_0_PORT,Advertise_0_NUM);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);  
    
    /* Restart BLE advertisement */
    RestartBleAvertisement();
}

/*******************************************************************************
* Function Name: void Cy_RTC_Alarm1Interrupt(void)
********************************************************************************
*
* Summary:
*  This function is the handler for the RTC Alarm1 Interrupt. It clears the 
*  interrupt and schedules a display refresh
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Cy_RTC_Alarm1Interrupt(void)
{   
    /* Clear any pending interrupts */
    Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
    __NVIC_ClearPendingIRQ(RTC_RTC_IRQ_cfg.intrSrc);
    
    /* Schedule a display refresh at the next available slot */
    RequestDisplayRefresh();
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
*******************************************************************************/
void InitLowPower(void)
{
    /* Initialize MCDWT */
    Cy_MCWDT_Init(MCWDT_HW, &MCWDT_config);
    
    /* Enable the interrupts for MCDWT Counter1 only */
    Cy_MCWDT_SetInterruptMask(MCWDT_HW, CY_MCWDT_CTR0);
    
    /* Initialize and enable MCWDT interrupt */
    Cy_SysInt_Init(&isr_MCWDT_cfg,IsrMcwdt);
    NVIC_ClearPendingIRQ(isr_MCWDT_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_MCWDT_cfg.intrSrc);
    
    /* Start MCDWT Counter0 */
    Cy_MCWDT_Enable(MCWDT_HW, CY_MCWDT_CTR0,0u);
    
    /* Initialize and enable the GPIO interrupt assigned to CM4 */
    Cy_SysInt_Init(&isr_gpio_cfg,isrGPIO);
    NVIC_ClearPendingIRQ(isr_gpio_cfg.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)isr_gpio_cfg.intrSrc);
    
    /* Start the RTC */
    RTC_Start();  
    
    /* Clear any pending interrupts */
    Cy_RTC_ClearInterrupt(CY_RTC_INTR_ALARM1);
    __NVIC_ClearPendingIRQ(RTC_RTC_IRQ_cfg.intrSrc);
    
    /*Configures the source (Alarm1) that trigger the interrupts */
    Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM1);

    /* Set the default date and time */
    Cy_RTC_SetDateAndTimeDirect(TIME_AT_RESET);
    
    /* Wait for 3 LFCLK cycles before the next RTC operation */
    Cy_SysLib_DelayUs(WAIT_3LFCLKS);
     
    /* Set alarm that generates interrupts at 1 minute intervals */
    Cy_RTC_SetAlarmDateAndTime(&alarm,CY_RTC_ALARM_1);
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
*******************************************************************************/
void HandleLowPowerMode(void)
{
    /* Flag used to check if MCWDT has been disabled */
    bool mcwdtDisabled = false;
    
    /* Make sure that BLE is ready to enter low power mode */
    if(IsBleReadyForLowPowerMode() == true)
    {  
        /* Disable MCDWT Counter0 to prevent periodic wake-ups if none of the
           status LEDs need periodic toggle and BLE is not connected */
        if((IsLEDperiodicToggleActive() == false)&&
           (IsBleconnected() == false))
        {
            Cy_MCWDT_Disable(MCWDT_HW, CY_MCWDT_CTR0,0u);
            NVIC_DisableIRQ((IRQn_Type)isr_MCWDT_cfg.intrSrc);
            mcwdtDisabled = true;
        }
        
        /* Enter Deep-sleep mode */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);

        /* Re-enable MCDWT Counter0 */
        if(mcwdtDisabled)
        {
            NVIC_EnableIRQ((IRQn_Type)isr_MCWDT_cfg.intrSrc);
            Cy_MCWDT_Enable(MCWDT_HW, CY_MCWDT_CTR0,0u);
        }
    }
}
/* [] END OF FILE */
